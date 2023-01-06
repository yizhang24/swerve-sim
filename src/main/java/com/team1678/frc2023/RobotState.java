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
    
    private static final double HISTORY_LENGTH_SECONDS = 0.5; // 1.0 / 0.02 = 50 entries at optimal loop times

    private final TreeMap<Double, Pose2d> drivetrainData = new TreeMap<>(); // Pose estimate from odometry
    private final TreeMap<Double, Pose2d> visionData = new TreeMap<>(); // Pose estimate from vision

    private Pose2d firstPose = new Pose2d();
    private final TreeMap<Double, Pose2d> correctedPoseData = new TreeMap<>(); // Real robot pose

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

                Pose2d pose = firstPose;

                // Iterate through all drivetrain observations
                for (Map.Entry<Double, Pose2d> drivetrainEntry : drivetrainData.entrySet()) {

                    // Find timestamp of next drivetrain update
                    Double nextDrivetrainObservation = drivetrainData.higherKey(drivetrainEntry.getKey());
                    if (nextDrivetrainObservation == null) {
                        nextDrivetrainObservation = drivetrainEntry.getKey() + Constants.kLooperDt;
                    }

                    Transform2d odometryDelta = new Transform2d();

                    double visionWeight;

                    // Get previous pose & calculate delta
                    Pose2d nextPose = drivetrainData.get(nextDrivetrainObservation);
                    if (nextPose != null) {
                        odometryDelta = nextPose.minus(drivetrainEntry.getValue());  
                    }
                    // Some weighting calculation?
                    visionWeight = 0.04;
            
                    // Create submap of vision entries between now and next drivetrain update
                    SortedMap<Double, Pose2d> intermediateVisionEntries = visionData.subMap(drivetrainEntry.getKey(), nextDrivetrainObservation);
                    
                    // Iterate through intermediate vision entries
                    for (Map.Entry<Double, Pose2d> visionEntry : intermediateVisionEntries.entrySet()) {
                        // Mix tracked current pose with vision data based on weighting
                        pose = pose.interpolate(visionEntry.getValue(), visionWeight);
                    }

                    pose = pose.plus(odometryDelta);
                }
                System.out.println(pose);
                System.out.println("------------------------------");
                correctedPoseData.put(timestamp, pose);
            }

            @Override
            public void onStop(double timestamp) {
                
            }
        };
        enabledLooper.register(mLoop);
    }

    public Pose2d getCorrectedPose(double timestamp) {
        Pose2d pose = getInterpolated(correctedPoseData, timestamp);
        return pose;
    }

    private void pruneByTime(double now) {
        double earliest = now - HISTORY_LENGTH_SECONDS;
        while (visionData.size() > 0 && visionData.firstKey() < earliest) {
            visionData.pollFirstEntry();
        }

        while (drivetrainData.size() > 0 && drivetrainData.firstKey() < earliest) {
            drivetrainData.pollFirstEntry();  
        }

        while (correctedPoseData.size() > 0 && correctedPoseData.firstKey() < earliest) {
            correctedPoseData.pollFirstEntry();
            firstPose = correctedPoseData.firstEntry().getValue(); 
        }
    }

    private Pose2d getInterpolated(TreeMap<Double, Pose2d> data, double timestamp) {
        Pose2d hasValue = data.get(timestamp);

        if (hasValue != null){
            return hasValue;
        } 

        Map.Entry<Double, Pose2d> earlier = data.floorEntry(timestamp);
        Map.Entry<Double, Pose2d> later = data.ceilingEntry(timestamp);

        if (earlier == null && later == null) {
            return null;
        } else if (earlier == null) {
            return data.firstEntry().getValue();
        } else if (later == null) {
            return data.lastEntry().getValue();
        }

        double scalar = (earlier.getKey() + timestamp) / (later.getKey() - earlier.getKey());
        return earlier.getValue().interpolate(later.getValue(), scalar);
    }
}
