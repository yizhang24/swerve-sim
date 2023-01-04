package com.team1678.frc2023.subsystems;

import com.team1678.frc2023.loops.Loop;
import com.team1678.frc2023.loops.ILooper;
import com.team1678.frc2023.Constants;
import com.team1678.frc2023.RobotState;
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

            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }


    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void stop() {
    }
}
