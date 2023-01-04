package com.team1678.lib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class ModuleState extends SwerveModulePosition {

    public double speedMetersPerSecond;

    public ModuleState(double distanceMeters, Rotation2d angle, double speedMetersPerSecond) {
        super(distanceMeters, angle);
        this.speedMetersPerSecond = speedMetersPerSecond;
    }
}
