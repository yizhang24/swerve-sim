package com.team1678.frc2022.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.sim.PhysicsSim;
import com.team1678.lib.math.Conversions;
import com.team1678.lib.util.CTREConfigs;
import com.team1678.lib.util.CTREModuleState;
import com.team1678.lib.util.SwerveModuleConstants;
import com.team254.lib.drivers.TalonFXFactory;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.WPI_CANCoder;

public class SimSwerveModule {
    public int moduleNumber;
    public double angleOffset;
    private WPI_TalonFX mAngleMotor;
    private WPI_TalonFX mDriveMotor;
    private WPI_CANCoder angleEncoder;
    private double lastAngle;

    private double anglekP;
    private double anglekI;
    private double anglekD;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SwerveConstants.driveKS,
            Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);

    public SimSwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        angleEncoder = new WPI_CANCoder(moduleConstants.cancoderID, "rio");
        configAngleEncoder();
        angleEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 255);
        angleEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 255);

        /* Angle Motor Config */
        mAngleMotor = TalonFXFactory.createWPIDefaultTalon(moduleConstants.angleMotorID);
        configAngleMotor();
        TalonFXConfiguration angleConfiguration = CTREConfigs.swerveAngleFXConfig();
        anglekP = angleConfiguration.slot0.kP;
        anglekI = angleConfiguration.slot0.kI;
        anglekD = angleConfiguration.slot0.kD;

        /* Drive Motor Config */
        mDriveMotor = TalonFXFactory.createWPIDefaultTalon(moduleConstants.driveMotorID);
        configDriveMotor();

        lastAngle = getState().angle.getDegrees();

        // Add to physics sim
        PhysicsSim.getInstance().addTalonFX(mAngleMotor, 0.5, 6800);
        PhysicsSim.getInstance().addTalonFX(mDriveMotor, 0.5, 6800);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); // Custom optimize command, since
                                                                                 // default WPILib optimize assumes
                                                                                 // continuous controller which CTRE is
                                                                                 // not

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond,
                    Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle.getDegrees(); // Prevent rotating module if speed is less then 1%. Prevents
                                                   // Jittering.
        mAngleMotor.set(ControlMode.Position,
                Conversions.degreesToFalcon(angle, Constants.SwerveConstants.angleGearRatio));
        lastAngle = angle;
    }

    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset,
                Constants.SwerveConstants.angleGearRatio);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder() {
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(CTREConfigs.swerveCancoderConfig());
    }

    private void configAngleMotor() {
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configAllSettings(CTREConfigs.swerveAngleFXConfig());
        mAngleMotor.setInverted(Constants.SwerveConstants.angleMotorInvert);
        mAngleMotor.setNeutralMode(Constants.SwerveConstants.angleNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(CTREConfigs.swerveDriveFXConfig());
        mDriveMotor.setInverted(Constants.SwerveConstants.driveMotorInvert);
        mDriveMotor.setNeutralMode(Constants.SwerveConstants.driveNeutralMode);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    public void updateAnglePID(double kP, double kI, double kD) {
        if (anglekP != kP) {
            anglekP = kP;
            mAngleMotor.config_kP(0, anglekP, Constants.kLongCANTimeoutMs);
        }
        if (anglekI != kI) {
            anglekI = kI;
            mAngleMotor.config_kI(0, anglekI, Constants.kLongCANTimeoutMs);
        }
        if (anglekD != kP) {
            anglekD = kD;
            mAngleMotor.config_kD(0, anglekD, Constants.kLongCANTimeoutMs);
        }
    }

    public double[] getAnglePIDValues() {
        double[] values = { anglekP, anglekI, anglekD };
        return values;
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public double getTargetAngle() {
        return lastAngle;
    }

    public SwerveModuleState getState() {
        double velocity = Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(),
                Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(),
                Constants.SwerveConstants.angleGearRatio));
        return new SwerveModuleState(velocity, angle);
    }

}
