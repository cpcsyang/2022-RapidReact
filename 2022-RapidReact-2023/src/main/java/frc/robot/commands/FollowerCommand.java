// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;


import org.frcteam2910.common.control.HolonomicMotionProfiledTrajectoryFollower;
import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.util.DrivetrainFeedforwardConstants;
import org.frcteam2910.common.util.HolonomicDriveSignal;
import org.frcteam2910.common.util.HolonomicFeedforward;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.utils.Conversions;

public class FollowerCommand extends CommandBase {

  public static final DrivetrainFeedforwardConstants FEEDFORWARD_CONSTANTS = new DrivetrainFeedforwardConstants(
      2.15, // 3.25
      0.00, // 0.4
      0.2);

  private final HolonomicMotionProfiledTrajectoryFollower follower = new HolonomicMotionProfiledTrajectoryFollower(
      new PidConstants(5, 0.0, 0.0),
      new PidConstants(5, 0.0, 0.0),
      new HolonomicFeedforward(FEEDFORWARD_CONSTANTS));

  private Trajectory m_trajectory;

  private DrivetrainSubsystem m_subsystem;

  /** Creates a new Follower. */
  public FollowerCommand(DrivetrainSubsystem subsystem, Trajectory trajectory) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    m_subsystem = subsystem;
    m_trajectory = trajectory;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    follower.follow(m_trajectory);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Optional<HolonomicDriveSignal> m_holonomicSignal = follower.update(
        Conversions.poseToRigidTransform(m_subsystem.getPose()),
        Conversions.chassisSpeedsToVector(m_subsystem.getVelocity()),
        m_subsystem.getVelocity().omegaRadiansPerSecond,
        Timer.getFPGATimestamp(), .02);

    if (m_holonomicSignal.isPresent()) {
      HolonomicDriveSignal driveSignal = m_holonomicSignal.get();
      if (m_holonomicSignal.get().isFieldOriented()) {

        m_subsystem.drive(
          ChassisSpeeds.fromFieldRelativeSpeeds(
            driveSignal.getTranslation().x / DrivetrainSubsystem.MAX_VOLTAGE * m_subsystem.getMaxVelocity(),
            driveSignal.getTranslation().y / DrivetrainSubsystem.MAX_VOLTAGE * m_subsystem.getMaxVelocity(),
            driveSignal.getRotation()  / DrivetrainSubsystem.MAX_VOLTAGE * m_subsystem.getMaxAngularVelocity(),
            m_subsystem.getPose().getRotation()));

      } else {
        m_subsystem.drive(new ChassisSpeeds(
          driveSignal.getTranslation().x / DrivetrainSubsystem.MAX_VOLTAGE * m_subsystem.getMaxVelocity(),
          driveSignal.getTranslation().y / DrivetrainSubsystem.MAX_VOLTAGE * m_subsystem.getMaxVelocity(),
          driveSignal.getRotation()  / DrivetrainSubsystem.MAX_VOLTAGE * m_subsystem.getMaxAngularVelocity()));
      }
    }

    // follower.update(/*m_subsystem.getPose()*/ new RigidTransform2(new
    // Vector2d(m_subsystem.getPose().getTranslation().getX(),
    // m_subsystem.getPose(), m_subsystem.getPose().getRotation()),
    // new Vector2d(m_subsystem.getVelocity().vxMetersPerSecond,
    // m_subsystem.getVelocity().vyMetersPerSecond),
    // m_subsystem.getVelocity().omegaRadiansPerSecond,
    // Timer.getFPGATimestamp(), .02);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return follower.getCurrentTrajectory().isEmpty();
  }
}
