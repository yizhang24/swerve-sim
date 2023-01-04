package com.team1678.frc2023.shuffleboard;


import com.team1678.frc2023.Constants;
import com.team1678.frc2023.subsystems.Swerve;
import com.team1678.lib.ModuleState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldView {
    private Field2d mField2d = new Field2d();
    private Swerve mSwerve = Swerve.getInstance();

    private Pose2d[] modulePoses = new Pose2d[4];
    private Pose2d robotPose = new Pose2d();

    public FieldView() {
        SmartDashboard.putData(mField2d);
    }

    private void updateSwervePoses() {
        if(mSwerve.getPose() != null) robotPose = mSwerve.getPose();
        else robotPose = new Pose2d();

        ModuleState[] moduleStates = mSwerve.getStates();

        for (int i = 0; i < modulePoses.length; i++) {
            Translation2d updatedPosition = Constants.SwerveConstants.swerveModuleLocations[i]
                    .rotateBy(robotPose.getRotation()).plus(robotPose.getTranslation());
            Rotation2d updatedRotation = mSwerve.getStates()[i].angle.plus(robotPose.getRotation());
            if(moduleStates[i].speedMetersPerSecond < 0.0) {
                updatedRotation = updatedRotation.plus(Rotation2d.fromDegrees(180));;
            }
            modulePoses[i] = new Pose2d(updatedPosition, updatedRotation);
        }
    }

    public void update() {
        updateSwervePoses();

        mField2d.setRobotPose(robotPose);
        mField2d.getObject("Swerve Modules").setPoses(modulePoses);
    }
}
