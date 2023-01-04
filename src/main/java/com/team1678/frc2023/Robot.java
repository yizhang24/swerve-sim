// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1678.frc2023;

import java.util.Optional;

import com.team1678.frc2023.auto.AutoModeExecutor;
import com.team1678.frc2023.auto.AutoModeSelector;
import com.team1678.frc2023.auto.modes.AutoModeBase;
import com.team1678.frc2023.controlboard.ControlBoard;
import com.team1678.frc2023.controlboard.ControlBoard.SwerveCardinal;
import com.team1678.frc2023.logger.LoggingSystem;
import com.team1678.frc2023.loops.CrashTracker;
import com.team1678.frc2023.loops.Looper;
import com.team1678.frc2023.shuffleboard.ShuffleBoardInteractions;
import com.team1678.frc2023.sim.PhysicsSim;
import com.team1678.frc2023.subsystems.Limelight;
import com.team1678.frc2023.subsystems.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends edu.wpi.first.wpilibj.TimedRobot {
	/**
	 * This function is run when the robot is first started up and should be used
	 * for any
	 * initialization code.
	 */

	 
	// instantiate enabled and disabled loopers
	private final Looper mEnabledLooper = new Looper();
	private final Looper mDisabledLooper = new Looper();
	// instantiate logging looper
	private final Looper mLoggingLooper = new Looper();

	// declare necessary class objects
	private ShuffleBoardInteractions mShuffleBoardInteractions;

	// subsystem instances
	private final ControlBoard mControlBoard = ControlBoard.getInstance();

	private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
	private final Swerve mSwerve = Swerve.getInstance();
	private final Limelight mLimelight = Limelight.getInstance();

	// auto instances
	private AutoModeExecutor mAutoModeExecutor;
	private AutoModeSelector mAutoModeSelector = new AutoModeSelector();

	public Robot() {
		CrashTracker.logRobotConstruction();
	}

	@Override
	public void robotInit() {

		mShuffleBoardInteractions = ShuffleBoardInteractions.getInstance();

		try {
			CrashTracker.logRobotInit();

			mSubsystemManager.setSubsystems(			
					mSwerve,
					mLimelight
			);

			mSubsystemManager.registerEnabledLoops(mEnabledLooper);
			mSubsystemManager.registerDisabledLoops(mDisabledLooper);
			
			mSwerve.resetOdometry(new Pose2d());
			mSwerve.resetAnglesToAbsolute();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void robotPeriodic() {
		mEnabledLooper.outputToSmartDashboard();
		mShuffleBoardInteractions.update();
	}

	@Override
	public void autonomousInit() {
		CrashTracker.logAutoInit();

		try {
			mDisabledLooper.stop();
			mEnabledLooper.start();
			mLoggingLooper.start();

			Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
			if (autoMode.isPresent()) {
				mSwerve.resetOdometry(autoMode.get().getStartingPose());
			}

			mAutoModeExecutor.start();

			mLimelight.setPipeline(Constants.VisionConstants.kDefaultPipeline);

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}

	}

	@Override
	public void autonomousPeriodic() {
		mLimelight.setLed(Limelight.LedMode.ON);
	}

	@Override
	public void teleopInit() {
		try {

			if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

			mSwerve.resetOdometry(new Pose2d(3, 3, new Rotation2d()));

			mDisabledLooper.stop();
			mEnabledLooper.start();
			mLoggingLooper.start();

			mLimelight.setLed(Limelight.LedMode.ON);
            mLimelight.setPipeline(Constants.VisionConstants.kDefaultPipeline);

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void teleopPeriodic() {
		try {

			if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

			mLimelight.setLed(Limelight.LedMode.ON);
			mLimelight.outputTelemetry();

			/* SWERVE DRIVE */
			// hold left bumper
			if (mControlBoard.getBrake()) {
				mSwerve.setLocked(true);
			} else {
				mSwerve.setLocked(false);
			}

			if (mControlBoard.zeroGyro()) {
				mSwerve.zeroGyro();
			}

			if (mControlBoard.getSwerveSnap() != SwerveCardinal.NONE) {
				mSwerve.startSnap(mControlBoard.getSwerveSnap().degrees);
			}
			Translation2d swerveTranslation = new Translation2d(mControlBoard.getSwerveTranslation().x(),
					mControlBoard.getSwerveTranslation().y());
			double swerveRotation = mControlBoard.getSwerveRotation();

			if (mControlBoard.getClimbAlign()) {
				mSwerve.angleAlignDrive(swerveTranslation, 270, true);
			} else if (mControlBoard.getVisionAlign()) {
				mSwerve.visionAlignDrive(swerveTranslation, true);
			} else {
				mSwerve.drive(swerveTranslation, swerveRotation, true, true);
			}

		} catch (Throwable t) {
			t.printStackTrace();
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledInit() {
		try {
			CrashTracker.logDisabledInit();
			mEnabledLooper.stop();
			mDisabledLooper.start();

			mLoggingLooper.stop();

			mLimelight.setLed(Limelight.LedMode.ON);
            mLimelight.triggerOutputs();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}

		if (mAutoModeExecutor != null) {
			mAutoModeExecutor.stop();
		}

		// Reset all auto mode state.
		mAutoModeSelector.reset();
		mAutoModeSelector.updateModeCreator();
		mAutoModeExecutor = new AutoModeExecutor();

	}

	@Override
	public void disabledPeriodic() {
		try {

			mDisabledLooper.outputToSmartDashboard();

			mAutoModeSelector.updateModeCreator();
			
			mSwerve.resetAnglesToAbsolute();

			mLimelight.setLed(Limelight.LedMode.ON);
			mLimelight.writePeriodicOutputs();
			mLimelight.outputTelemetry();

			if (DriverStation.isFMSAttached() &&  !LoggingSystem.inCompetition) {
				NetworkTable nt = NetworkTableInstance.getDefault().getTable("FMSInfo");
				String eventName = nt.getEntry("EventName").getString("NA");
				int matchNumber = (int) nt.getEntry("MatchNumber").getDouble(0.0);
				int matchTypeIndex = (int) nt.getEntry("MatchType").getDouble(0.0);
				System.out.println(matchTypeIndex);
				String matchTypeName;
				switch (matchTypeIndex) {
					case 1:
						matchTypeName = "Practice";
						break;
					case 2:
				  		matchTypeName = "Qualification";
						break;
					case 3:
				  		matchTypeName = "Elimination";
						break;
				  	default:
						matchTypeName = "None";
						break;
				}

				LoggingSystem.getInstance().updateMatchInfo(eventName, matchTypeName, matchNumber);
			}

			Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
			if (autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
				System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
				mAutoModeExecutor.setAutoMode(autoMode.get());
			}

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testInit() {
		try {
			mDisabledLooper.stop();
			mEnabledLooper.stop();

			mLoggingLooper.stop();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void simulationPeriodic() {
		PhysicsSim.getInstance().run();
		mSwerve.updateSim();
	}
}
