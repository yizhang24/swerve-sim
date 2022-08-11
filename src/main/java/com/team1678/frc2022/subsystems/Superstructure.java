package com.team1678.frc2022.subsystems;

import com.team1678.frc2022.loops.Loop;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.RobotState;
import com.team254.lib.vision.AimingParameters;

import com.team254.lib.geometry.Pose2d;

import java.util.Optional;

public class Superstructure extends Subsystem {

    // superstructure instance
    private static Superstructure mInstance;

    public synchronized static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    };

    /*** REQUIRED INSTANCES ***/
    private final Swerve mSwerve = Swerve.getInstance();
    private final Limelight mLimelight = Limelight.getInstance();

    // robot state
    private final RobotState mRobotState = RobotState.getInstance();

    // timer for reversing the intake and then stopping it once we have two correct

    // aiming parameter vars
    private Optional<AimingParameters> real_aiming_params_ = Optional.empty();
    private int mTrackId = -1;
    private double mTargetAngle = 0.0;
    private double mCorrectedDistanceToTarget = 0.0;

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onLoop(double timestamp) {
                updateVisionAimingParameters();
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }


    /***
     * GET REAL AIMING PARAMETERS
     * called in updateVisionAimingSetpoints()
     */
    public Optional<AimingParameters> getRealAimingParameters() {
        Optional<AimingParameters> aiming_params = RobotState.getInstance().getAimingParameters(mTrackId,
                Constants.VisionConstants.kMaxGoalTrackAge);
        if (aiming_params.isPresent()) {
            return aiming_params;
        } else {
            Optional<AimingParameters> default_aiming_params = RobotState.getInstance().getDefaultAimingParameters();
            return default_aiming_params;
        }
    }

    /*** UPDATE VISION AIMING PARAMETERS FROM GOAL TRACKING ***/
    public void updateVisionAimingParameters() {
        // get aiming parameters from either vision-assisted goal tracking or
        // odometry-only tracking
        real_aiming_params_ = getRealAimingParameters();

        // predicted pose and target
        Pose2d predicted_field_to_vehicle = mRobotState
                .getPredictedFieldToVehicle(Constants.VisionConstants.kLookaheadTime);
        Pose2d predicted_vehicle_to_goal = predicted_field_to_vehicle.inverse()
                .transformBy(real_aiming_params_.get().getFieldToGoal());

        // update align delta from target and distance from target
        mTrackId = real_aiming_params_.get().getTrackId();
        mTargetAngle = predicted_vehicle_to_goal.getTranslation().direction().getRadians() + Math.PI;

        // send vision aligning target delta to swerve
        mSwerve.acceptLatestGoalTrackVisionAlignGoal(mTargetAngle);

        // update distance to target
        if (mLimelight.hasTarget() && mLimelight.getLimelightDistanceToTarget().isPresent()) {
            mCorrectedDistanceToTarget = mLimelight.getLimelightDistanceToTarget().get();
        } else {
            mCorrectedDistanceToTarget = predicted_vehicle_to_goal.getTranslation().norm();
        }
    }
    
    // get vision align delta from goal
    public double getVisionAlignGoal() {
        return mTargetAngle;
    }

    public double getDistanceToTarget() {
        return mCorrectedDistanceToTarget;
    }

    // check if our limelight sees a vision target
    public boolean hasTarget() {
        return mLimelight.hasTarget();
    }

    // checked if we are vision aligned to the target within an acceptable horiz. error
    public boolean isAimed() {
        return mLimelight.isAimed();
    }


    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void stop() {
    }
}
