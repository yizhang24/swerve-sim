package com.team1678.frc2023.drivers;

import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.Pigeon2;
import com.team1678.frc2023.Constants;
import com.team1678.frc2023.Ports;

import edu.wpi.first.math.geometry.Rotation2d;

public class Pigeon {

    private static Pigeon mInstance;

    public static Pigeon getInstance() {
        if (mInstance == null) {
            mInstance = new Pigeon(Ports.PIGEON);
        }
        return mInstance;
    }

    // Actual pigeon object
    private final Pigeon2 mGyro;

    // Configs
    private boolean inverted = Constants.SwerveConstants.invertGyro;
    private Rotation2d yawAdjustmentAngle = new Rotation2d();
    private Rotation2d rollAdjustmentAngle = new Rotation2d();

    private Pigeon(int port) {        
        mGyro = new Pigeon2(port, "canivore1");
        mGyro.configFactoryDefault();
    }

    public Rotation2d getYaw() {
        Rotation2d angle = getUnadjustedYaw().minus(yawAdjustmentAngle);
        if (inverted) {
            angle = angle.unaryMinus();
        }
        return angle;
    }

    public Rotation2d getRoll() {
        return getUnadjustedRoll().minus(rollAdjustmentAngle);
    }

    /**
     * Sets the yaw register to read the specified value.
     *
     * @param angleDeg New yaw in degrees
     */
    public void setYaw(double angleDeg) {
        yawAdjustmentAngle = getUnadjustedYaw().rotateBy(Rotation2d.fromDegrees(angleDeg).unaryMinus());
    }

    /**
     * Sets the roll register to read the specified value.
     *
     * @param angleDeg New yaw in degrees
     */
    public void setRoll(double angleDeg) {
        rollAdjustmentAngle = getUnadjustedRoll().rotateBy(Rotation2d.fromDegrees(angleDeg).unaryMinus());
    }

    public Rotation2d getUnadjustedYaw() {
        return Rotation2d.fromDegrees(mGyro.getYaw());
    }

    public Rotation2d getUnadjustedPitch() {
        return Rotation2d.fromDegrees(mGyro.getPitch());
    }

    public Rotation2d getUnadjustedRoll() {
        return Rotation2d.fromDegrees(mGyro.getRoll());
    }

    public BasePigeonSimCollection getSimPigeon() {
        return mGyro.getSimCollection();
    }
} 