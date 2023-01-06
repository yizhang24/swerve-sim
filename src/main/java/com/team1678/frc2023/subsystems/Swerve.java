package com.team1678.frc2023.subsystems;

import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.team1678.frc2023.Constants;
import com.team1678.frc2023.Robot;
import com.team1678.frc2023.RobotState;
import com.team1678.frc2023.drivers.Pigeon;
import com.team1678.frc2023.drivers.SimSwerveModule;
import com.team1678.frc2023.logger.Log;
import com.team1678.frc2023.logger.LoggingSystem;
import com.team1678.frc2023.loops.ILooper;
import com.team1678.frc2023.loops.Loop;
import com.team1678.lib.ModuleState;
import com.team254.lib.util.TimeDelayedBoolean;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Swerve extends Subsystem {

    private static Swerve mInstance;

    public PeriodicIO mPeriodicIO = new PeriodicIO();

    // status variable for being enabled
    public boolean mIsEnabled = false;

    // limelight instance for raw aiming
    Limelight mLimelight = Limelight.getInstance();

    // wants vision aim during auto
    public boolean mWantsAutoVisionAim = false;

    public SwerveDriveOdometry mSwerveOdometry;
    public SimSwerveModule[] mSwerveMods;

    public Pigeon mPigeon = Pigeon.getInstance();

    BasePigeonSimCollection mSimPigeon = mPigeon.getSimPigeon();

    // chassis velocity status
    ChassisSpeeds chassisVelocity = new ChassisSpeeds();

    public boolean isSnapping;
    private double mLimelightVisionAlignGoal;
    private double mGoalTrackVisionAlignGoal;
    private double mVisionAlignAdjustment;

    private RobotState mRobotState = RobotState.getInstance();

    public ProfiledPIDController snapPIDController;
    public PIDController visionPIDController;
    
    // Private boolean to lock Swerve wheels
    private boolean mLocked = false;

    // Getter
    public boolean getLocked() {
        return mLocked;
    }
    // Setter
    public void setLocked(boolean lock) {
        mLocked = lock;
    }

    public static Swerve getInstance() {
        if (mInstance == null) {
            mInstance = new Swerve();
        }
        return mInstance;
    }

    public Swerve() {        
        snapPIDController = new ProfiledPIDController(Constants.SnapConstants.kP,
                                                      Constants.SnapConstants.kI, 
                                                      Constants.SnapConstants.kD,
                                                      Constants.SnapConstants.kThetaControllerConstraints);
        snapPIDController.enableContinuousInput(-Math.PI, Math.PI);

        visionPIDController = new PIDController(Constants.VisionAlignConstants.kP,
                                                        Constants.VisionAlignConstants.kI,
                                                        Constants.VisionAlignConstants.kD);
        visionPIDController.enableContinuousInput(-Math.PI, Math.PI);
        visionPIDController.setTolerance(0.0);

        zeroGyro();

        mSwerveMods = new SimSwerveModule[] {
            new SimSwerveModule(0, Constants.SwerveConstants.Mod0.constants),
            new SimSwerveModule(1, Constants.SwerveConstants.Mod1.constants),
            new SimSwerveModule(2, Constants.SwerveConstants.Mod2.constants),
            new SimSwerveModule(3, Constants.SwerveConstants.Mod3.constants)
        };

        LoggingSystem.getInstance().registerObject(this.getClass(), this);

        mSwerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, mPigeon.getYaw(), getStates());
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mIsEnabled = true;
            }

            @Override
            public void onLoop(double timestamp) {
                mIsEnabled = false;
                chooseVisionAlignGoal();
                updateSwerveOdometry();
                addSimPoseObservations();
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    private final double kVisionUpdateFrequency = 0.5;
    private double kLastVisionUpdate = 0.0;

    public Pose2d kSimVisionPose = new Pose2d();

    private void addSimPoseObservations() {
        if (kVisionUpdateFrequency + kLastVisionUpdate < Timer.getFPGATimestamp()) {
            kSimVisionPose = getPose();
            mRobotState.addVisionObservation(Timer.getFPGATimestamp(), kSimVisionPose);
            kLastVisionUpdate = Timer.getFPGATimestamp();
        }
        mRobotState.addOdometryObservation(Timer.getFPGATimestamp(), Constants.addNoise(getPose()));
    }
    
    public void setWantAutoVisionAim(boolean aim) {
        mWantsAutoVisionAim = aim;
    } 

    public boolean getWantAutoVisionAim() {
        return mWantsAutoVisionAim;
    }

    public void visionAlignDrive(Translation2d translation2d, boolean fieldRelative) {
        drive(translation2d, mVisionAlignAdjustment, fieldRelative, false);
    }

    public void angleAlignDrive(Translation2d translation2d, double targetHeading, boolean fieldRelative) {
        snapPIDController.setGoal(new TrapezoidProfile.State(Math.toRadians(targetHeading), 0.0));
        double angleAdjustment = snapPIDController.calculate(mPigeon.getYaw().getRadians());
        drive(translation2d, angleAdjustment, fieldRelative, false);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        if (isSnapping) {
            if (Math.abs(rotation) == 0.0) {
                maybeStopSnap(false);
                rotation = calculateSnapValue();
            } else {
                maybeStopSnap(true);
            }
        }
        SwerveModuleState[] SwerveModuleStates = null;
        if (mLocked) {
            SwerveModuleStates = new SwerveModuleState[]{
                new SwerveModuleState(0.1, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0.1, Rotation2d.fromDegrees(315)),
                new SwerveModuleState(0.1, Rotation2d.fromDegrees(135)),
                new SwerveModuleState(0.1, Rotation2d.fromDegrees(225))
            };
        } else {
            SwerveModuleStates =
                Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
                    fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                        translation.getX(), 
                                        translation.getY(), 
                                        rotation, 
                                        mPigeon.getYaw().rotateBy(Rotation2d.fromDegrees(-90))
                                    )
                                    : new ChassisSpeeds(
                                        translation.getX(), 
                                        translation.getY(), 
                                        rotation)
                                    );
        }
        SwerveDriveKinematics.desaturateWheelSpeeds(SwerveModuleStates, Constants.SwerveConstants.maxSpeed);

        for (SimSwerveModule mod : mSwerveMods) {
            mod.setDesiredState(SwerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void acceptLatestGoalTrackVisionAlignGoal(double vision_goal) {
        mGoalTrackVisionAlignGoal = vision_goal; 
    }

    public void chooseVisionAlignGoal() {
        double currentAngle = mPigeon.getYaw().getRadians();
        if (mLimelight.hasTarget()) {
            double targetOffset = Math.toRadians(mLimelight.getOffset()[0]);
            mLimelightVisionAlignGoal = MathUtil.inputModulus(currentAngle - targetOffset, 0.0, 2 * Math.PI);
            visionPIDController.setSetpoint(mLimelightVisionAlignGoal);
        } else {
            visionPIDController.setSetpoint(mGoalTrackVisionAlignGoal);
        }

        mVisionAlignAdjustment = visionPIDController.calculate(currentAngle);
    }

    public double calculateSnapValue() {
        return snapPIDController.calculate(mPigeon.getYaw().getRadians());
    }

    public void startSnap(double snapAngle) {
        snapPIDController.reset(mPigeon.getYaw().getRadians());
        snapPIDController.setGoal(new TrapezoidProfile.State(Math.toRadians(snapAngle), 0.0));
        isSnapping = true;
    }
    
    TimeDelayedBoolean delayedBoolean = new TimeDelayedBoolean();

    private boolean snapComplete() {
        double error = snapPIDController.getGoal().position - mPigeon.getYaw().getRadians();
        return delayedBoolean.update(Math.abs(error) < Math.toRadians(Constants.SnapConstants.kEpsilon), Constants.SnapConstants.kTimeout);
    }

    public void maybeStopSnap(boolean force){
        if (!isSnapping) {
            return;
        } 
        if (force || snapComplete()) {
            isSnapping = false;
            snapPIDController.reset(mPigeon.getYaw().getRadians());
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);
        
        for(SimSwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
            SmartDashboard.putNumber("mod " + mod.moduleNumber +  " desired speed", desiredStates[mod.moduleNumber].speedMetersPerSecond);
            SmartDashboard.putNumber("mod " + mod.moduleNumber +  " desired angle", MathUtil.inputModulus(desiredStates[mod.moduleNumber].angle.getDegrees(), 0, 180));
        }
    }    

    public Pose2d getPose() {
        return mSwerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        mSwerveOdometry.resetPosition(mPigeon.getYaw(), getStates(), pose);
        zeroGyro(pose.getRotation().getDegrees());
    }

    public void resetAnglesToAbsolute() {
        for (SimSwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public ModuleState[] getStates() {
        ModuleState[] states = new ModuleState[4];
        for(SimSwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public void setAnglePIDValues(double kP, double kI, double kD) {
        for (SimSwerveModule SimSwerveModule : mSwerveMods) {
            SimSwerveModule.updateAnglePID(kP, kI, kD);
        }
    }

    public double[] getAnglePIDValues(int index) {
        return mSwerveMods[index].getAnglePIDValues();
    }

    public void setVisionAlignPIDValues(double kP, double kI, double kD) {
        visionPIDController.setPID(kP, kI, kD);
    }

    public double[] getVisionAlignPIDValues() {
        return  new double[] {visionPIDController.getP(), visionPIDController.getI(), visionPIDController.getD()};
    }

    @Override
    public void zeroSensors(){
        zeroGyro(0.0);
    }
    
    public void zeroGyro(){
        zeroGyro(0.0);
    }

    public void zeroGyro(double reset){
        mPigeon.setYaw(reset);
        visionPIDController.reset();
    }

    public void updateSwerveOdometry(){
        mSwerveOdometry.update(mPigeon.getYaw(), getStates());

        chassisVelocity = Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(
                    mInstance.mSwerveMods[0].getSpeed(),
                    mInstance.mSwerveMods[1].getSpeed(),
                    mInstance.mSwerveMods[2].getSpeed(),
                    mInstance.mSwerveMods[3].getSpeed()
            );
    }

    @Override
    public void stop() {
        mIsEnabled = false;
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.odometry_pose_x = mSwerveOdometry.getPoseMeters().getX();
        mPeriodicIO.odometry_pose_y = mSwerveOdometry.getPoseMeters().getY();
        mPeriodicIO.odometry_pose_rot = mSwerveOdometry.getPoseMeters().getRotation().getDegrees();
        mPeriodicIO.pigeon_heading = mPigeon.getYaw().getDegrees();
        mPeriodicIO.robot_pitch = mPigeon.getUnadjustedPitch().getDegrees();
        mPeriodicIO.robot_roll = mPigeon.getRoll().getDegrees();
        mPeriodicIO.snap_target = Math.toDegrees(snapPIDController.getGoal().position);
        mPeriodicIO.vision_align_target_angle = Math.toDegrees(mLimelightVisionAlignGoal);
        mPeriodicIO.swerve_heading = MathUtil.inputModulus(mPigeon.getYaw().getDegrees(), 0, 360);
    }

    public static class PeriodicIO {
        // inputs
        public double odometry_pose_x;
        public double odometry_pose_y;
        public double odometry_pose_rot;

        public double pigeon_heading;
        public double robot_pitch;
        public double robot_roll;
        public double vision_align_target_angle;
        public double swerve_heading;

        public double angular_velocity;
        public double goal_velocity;

        public double profile_position;

        // outputs
        public double snap_target;

    }

    public void updateSim() {
        double chassisRotationSpeed = chassisVelocity.omegaRadiansPerSecond;

        mSimPigeon.addHeading(chassisRotationSpeed);
    }

    @Log
    private double getOdometryX() {
        return mPeriodicIO.odometry_pose_x;
    }

    @Log
    private double getOdometryY() {
        return mPeriodicIO.odometry_pose_x;
    }

    @Log
    private double getOdometryRot() {
        return mPeriodicIO.odometry_pose_x;
    }
    
    @Log
    private double timestamp() {
        return Timer.getFPGATimestamp(); 
    }

    public static class SwerveModuleConstants {
        public final int driveMotorID;
        public final int angleMotorID;
        public final int cancoderID;
        public final double angleOffset;
    
        public SwerveModuleConstants(int driveMotorID, int angleMotorID, int canCoderID, double angleOffset) {
            this.driveMotorID = driveMotorID;
            this.angleMotorID = angleMotorID;
            this.cancoderID = canCoderID;
            this.angleOffset = angleOffset;
        }
    }
}

