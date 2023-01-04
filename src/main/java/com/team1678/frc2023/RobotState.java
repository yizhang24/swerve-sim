package com.team1678.frc2023;

import com.team1678.frc2023.subsystems.Limelight;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import com.team254.lib.util.MovingAveragePose2d;
import com.team254.lib.vision.AimingParameters;
import com.team254.lib.vision.GoalTracker;
import com.team254.lib.vision.GoalTracker.TrackReportComparator;
import com.team254.lib.vision.TargetInfo;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.*;

public class RobotState {
    private static RobotState mInstance;
    public static RobotState getInstance() {
        if (mInstance == null) {
            mInstance = new RobotState();
        }

        return mInstance;
    }

    
}
