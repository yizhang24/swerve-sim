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
    // Singleton instance
    private static RobotState mInstance;
    public static RobotState getInstance() {
        if (mInstance == null) {
            mInstance = new RobotState();
        }

        return mInstance;
    }
    
    private static final double HISTORY_LENGTH_SECONDS = 1.0; // 1.0 / 0.02 = 50 entries at optimal loop times
    private static final double BASE_VISION_WEIGHT = 0.1; // How much of the weighted average should be vision per cycle
    
    // 1 - (1 - 0.1)^(1/0.02) = 94.486% pose is from vision after one second

    private final TreeMap<Double, Pose2d> drivetrainData = new TreeMap<>(); // Pose estimate from odometry
    private final TreeMap<Double, Pose2d> visionData = new TreeMap<>(); // Pose estimate from vision

    private final TreeMap<Double, Pose2d> correctedPoseData = new TreeMap<>(); // Calculated robot pose


    /**
     * Add a Pose2d observation from wheel odometry. 
     * @param timestamp Timestamp of observation.
     * @param field_to_vehicle Pose as reported by drivetrain.
     */
    public void addOdometryObservation(double timestamp, Pose2d field_to_vehicle) {
        drivetrainData.put(timestamp, field_to_vehicle);
    }

    /**
     * Add a Pose2d obesrvation from vision.
     * @param timestamp Timestamp of observation.
     * @param field_to_vehicle Pose as reported by vision localization.
     */
    public void addVisionObservation(double timestamp, Pose2d field_to_vehicle) {
        visionData.put(timestamp, field_to_vehicle);
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                
            }

            @Override
            public void onLoop(double timestamp) {
                pruneByTime(timestamp);

                // Get oldest recored pose to iterate on
                Pose2d pose = new Pose2d();
                if (correctedPoseData.firstEntry() != null) {
                    pose = correctedPoseData.firstEntry().getValue();
                }

                // Iterate through all drivetrain observations
                for (Map.Entry<Double, Pose2d> drivetrainEntry : drivetrainData.entrySet()) {

                    // Find timestamp of next drivetrain update
                    Double nextDrivetrainObservation = drivetrainData.higherKey(drivetrainEntry.getKey());
                    if (nextDrivetrainObservation == null) {
                        nextDrivetrainObservation = drivetrainEntry.getKey() + Constants.kLooperDt;
                    }

                    // Create submap of vision entries between now and next drivetrain update
                    SortedMap<Double, Pose2d> intermediateVisionEntries = visionData.subMap(drivetrainEntry.getKey(), nextDrivetrainObservation);
                    
                    
                    double visionWeight = BASE_VISION_WEIGHT;

                    // Get previous pose & calculate delta
                    Transform2d odometryDelta = new Transform2d();
                    Pose2d nextPose = drivetrainData.get(nextDrivetrainObservation);
                    if (nextPose != null) {
                        odometryDelta = nextPose.minus(drivetrainEntry.getValue());  
                    }            

                    // Iterate through intermediate vision entries
                    for (Map.Entry<Double, Pose2d> visionEntry : intermediateVisionEntries.entrySet()) {
                        // Mix tracked current pose with vision data based on weighting
                        pose = pose.interpolate(visionEntry.getValue(), visionWeight);
                    }

                    // Transform tracked pose by delta to next drivetrain observation
                    pose = pose.plus(odometryDelta);
                }

                // Add corrected pose to map
                correctedPoseData.put(timestamp, pose);
            }


            @Override
            public void onStop(double timestamp) {
                
            }
        });
    }

    /**
     * Get a pose from stored corrected poses, interpolating between observations as neccesary.
     * @param timestamp Timestamp to look for pose at.
     * @return Corrected pose at timestamp.
     */
    public Pose2d getCorrectedPose(double timestamp) {
        return getInterpolated(correctedPoseData, timestamp);
    }

    /**
     * Removes entries older than HISTORY_LENGTH_SECONDS ago.
     * @param now Current timestamp to reference.
     */
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
        }
    }

    /**
     * Linearly interpolates between Pose2d objects in a treemap to get positions between data points.
     * @param dataset Dataset to interpolate.
     * @param timestamp Timestamp to look for.
     * @return A Pose2d that is within the range of the dataset.
     */
    private Pose2d getInterpolated(TreeMap<Double, Pose2d> dataset, double timestamp) {
        Pose2d hasValue = dataset.get(timestamp);

        if (hasValue != null){
            return hasValue;
        } 

        Map.Entry<Double, Pose2d> earlier = dataset.floorEntry(timestamp);
        Map.Entry<Double, Pose2d> later = dataset.ceilingEntry(timestamp);

        if (earlier == null && later == null) {
            return null;
        } else if (earlier == null) {
            return dataset.firstEntry().getValue();
        } else if (later == null) {
            return dataset.lastEntry().getValue();
        }

        double scalar = (earlier.getKey() + timestamp) / (later.getKey() - earlier.getKey());
        return earlier.getValue().interpolate(later.getValue(), scalar);
    }
}
