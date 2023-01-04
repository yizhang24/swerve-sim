package com.team1678.frc2023.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class SwerveTrajectoryAction implements Action {
	private final Timer m_timer = new Timer();
	private final Trajectory m_trajectory;
	private final Supplier<Pose2d> m_pose;
	private final SwerveDriveKinematics m_kinematics;
	private final HolonomicDriveController m_controller;
	private final Consumer<SwerveModuleState[]> m_outputModuleStates;
	private final Supplier<Rotation2d> m_desiredRotation;

	public SwerveTrajectoryAction(Trajectory trajectory,
			Supplier<Pose2d> pose,
			SwerveDriveKinematics kinematics,
			PIDController xController,
			PIDController yController,
			ProfiledPIDController thetaController,
			Supplier<Rotation2d> desiredRotation,
			Consumer<SwerveModuleState[]> outputModuleStates) {
		m_trajectory = trajectory;
		m_pose = pose;
		m_kinematics = kinematics;
		m_controller = new HolonomicDriveController(xController, yController, thetaController);
		m_outputModuleStates = outputModuleStates;
		m_desiredRotation = desiredRotation;
	}

	public SwerveTrajectoryAction(
			Trajectory trajectory,
			Supplier<Pose2d> pose,
			SwerveDriveKinematics kinematics,
			PIDController xController,
			PIDController yController,
			ProfiledPIDController thetaController,
			Consumer<SwerveModuleState[]> outputModuleStates) {
		this(
				trajectory,
				pose,
				kinematics,
				xController,
				yController,
				thetaController,
				() -> trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation(),
				outputModuleStates);
	}

	@Override
	public boolean isFinished() {
		return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
	}

	@Override
	public void start() {
		m_timer.reset();
		m_timer.start();
	}

	@Override
	public void update() {
		double curTime = m_timer.get();
		var desiredState = m_trajectory.sample(curTime);
		var targetChassisSpeeds = m_controller.calculate(m_pose.get(), desiredState, m_desiredRotation.get());
		var targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);
		m_outputModuleStates.accept(targetModuleStates);
	}

	@Override
	public void done() {
		m_timer.stop();
		m_outputModuleStates.accept(m_kinematics.toSwerveModuleStates(new ChassisSpeeds()));
	}

	// get initial pose
	public Pose2d getInitialPose() {
		return m_trajectory.getInitialPose();
	}

}
