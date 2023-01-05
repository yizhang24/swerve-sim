package com.team1678.frc2023;

import java.util.Map;
import java.util.SortedMap;
import java.util.TreeMap;

import com.team1678.frc2023.loops.ILooper;
import com.team1678.frc2023.loops.Loop;
import com.team1678.frc2023.subsystems.Subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;

public class RobotState extends Subsystem {
    private static RobotState mInstance;
    public static RobotState getInstance() {
        if (mInstance == null) {
            mInstance = new RobotState();
        }

        return mInstance;
    }
    
    private static final double HISTORY_LENGTH_SECONDS = 1.0; // 1.0 / 0.02 = 50 entries at optimal loop time

    private final TreeMap<Double, Pose2d> drivetrainData = new TreeMap<>(); // Pose estimate from odometry
    private final TreeMap<Double, Pose2d> visionData = new TreeMap<>(); // Pose estimate from vision

    private final TreeMap<Double, Pose2d> correctedPoseData = new TreeMap<>();

    private boolean hasNewVisionUpdate = false;

    public void addOdometryObservation(double timestamp, Pose2d field_to_vehicle) {
        drivetrainData.put(timestamp, field_to_vehicle);
    }

    public void addVisionObservation(double timestamp, Pose2d field_to_vehicle) {
        visionData.put(timestamp, field_to_vehicle);
        hasNewVisionUpdate = true;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        Loop mLoop = new Loop() {
            @Override
            public void onStart(double timestamp) {
                
            }

            @Override
            public void onLoop(double timestamp) {
                pruneByTime(timestamp);
                Pose2d correctedPose = null;
                // Iterate through all drivetrain observations
                for (Map.Entry<Double, Pose2d> drivetrainEntry : drivetrainData.entrySet()) {

                    // Find timestamp of next drivetrain update
                    Double nextDrivetrainObservation = drivetrainData.higherKey(drivetrainEntry.getKey());
                    if (nextDrivetrainObservation == null) {
                        nextDrivetrainObservation = drivetrainEntry.getKey() + Constants.kLooperDt;
                    }

                    Transform2d odometryDelta = new Transform2d();

                    double visionWeight;

                    if (correctedPose == null) {
                        visionWeight = 1.0; // Completely trust vision on first update since vision is absolute
                        correctedPose = new Pose2d();
                    } else {
                        // Get previous pose & calculate delta
                        Pose2d nextPose = drivetrainData.get(nextDrivetrainObservation);
                        if (nextPose != null) {
                            odometryDelta = nextPose.minus(drivetrainEntry.getValue());  
                        }
                        // Some weighting calculation?
                        visionWeight = 0.0;
                    }
            
                    // Create submap of vision entries between now and next drivetrain update
                    SortedMap<Double, Pose2d> intermediateVisionEntries = visionData.subMap(drivetrainEntry.getKey(), nextDrivetrainObservation);
                    
                    // Iterate through intermediate vision entries
                    for (Map.Entry<Double, Pose2d> visionEntry : intermediateVisionEntries.entrySet()) {
                        // Mix tracked current pose with vision data based on weighting
                        correctedPose = correctedPose.interpolate(visionEntry.getValue(), visionWeight);
                    }

                    correctedPose.plus(odometryDelta);
                }
                correctedPoseData.put(timestamp, correctedPose);
            }

            @Override
            public void onStop(double timestamp) {
                
            }
        };
        enabledLooper.register(mLoop);
    }

    private void pruneByTime(double now) {
        double earliest = now - HISTORY_LENGTH_SECONDS;
        while (visionData.firstKey() < earliest && visionData.size() > 0) {
            visionData.pollFirstEntry();
        }

        while (drivetrainData.firstKey() < earliest && drivetrainData.size() > 0) {
            drivetrainData.pollFirstEntry();
        }

        while (correctedPoseData.firstKey() < earliest && correctedPoseData.size() > 0) {
            correctedPoseData.pollFirstEntry();
        }
    }

    public void getPose(double timestamp) {

    }

    private Pose2d getInterpolated(TreeMap<Double, Pose2d> data, double timestamp) {
        Pose2d hasValue = data.get(timestamp);
        if (hasValue != null) {
            return hasValue;
        }

        Map.Entry<Double, Pose2d> earlier = data.floorEntry(timestamp);
        Map.Entry<Double, Pose2d> later = data.ceilingEntry(timestamp);
        double scalar = (earlier.getKey() + timestamp) / (later.getKey() - earlier.getKey());
        return earlier.getValue().interpolate(later.getValue(), scalar);
    }
}
