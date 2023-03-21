package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class AutoDriveCommand extends CommandBase {

  private PIDController translationXController = new PIDController(1.2, 0, 0); //10
  private PIDController translationYController = new PIDController(0, 0, 0);
  private PIDController rotationController = new PIDController(0, 0, 0); // 1.2

  private Pose2d currentPose;
  //private double currentAngle;
  private double currentX;

  private double translationXSpeed;
  //private double rotationSpeed;

  private double targetX;

  private DrivetrainSubsystem drivetrainSubsystem;

  public AutoDriveCommand(DrivetrainSubsystem drivetrainSubsystem, Translation2d targetTranslation, Rotation2d targetRotation) {
      this.drivetrainSubsystem = drivetrainSubsystem;
      addRequirements(drivetrainSubsystem);
      
      targetX = targetTranslation.getX();

      translationXController.setSetpoint(targetTranslation.getX()); 
      translationYController.setSetpoint(targetTranslation.getY());

      rotationController.setSetpoint(targetRotation.getRadians());
      rotationController.enableContinuousInput(0, 2 * Math.PI);
  }

  @Override
  public void initialize(){
    drivetrainSubsystem.resetPosition();
  }

  @Override
  public void execute() {
    currentPose = drivetrainSubsystem.getPose();
    currentX = currentPose.getX();
    //currentAngle = currentPose.getRotation().getRadians();

    translationXSpeed = translationXController.calculate(currentX);
    //rotationSpeed = rotationController.calculate(currentAngle);

    drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(translationXSpeed, 0, 0, drivetrainSubsystem.getPose().getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {
    return (Math.abs(currentX - targetX) < 0.01);
  }
}