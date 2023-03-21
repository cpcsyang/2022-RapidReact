package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drivers.Pixy;
import frc.robot.subsystems.DrivetrainSubsystem;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

public class AutoFollowCargoCommand extends CommandBase {
  private DrivetrainSubsystem drivetrain;
  private Pixy pixy;
  private PIDController translationXPidController = new PIDController(0.001, 0, 0);
  private PIDController rotationPidController = new PIDController(0.025, 0, 0);
  
  private double lastXOffset = 0;
  private double currentXOffset = 0;

  private double rotationSpeed = drivetrain.maxAngularVelocity;
  private double translationXSpeed = drivetrain.maxVelocity;

  public AutoFollowCargoCommand(DrivetrainSubsystem drivetrain, Pixy pixy) {

    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    this.pixy = pixy;

    translationXPidController.setSetpoint(2000);
    rotationPidController.setSetpoint(155);
  }

  @Override
  public void initialize() {

    System.out.println("start following cargo");
    drivetrain.resetPosition();
    pixy.turnOnLights();
  }

  @Override
  public void execute() {
    Block cargo = pixy.getLargestBlock();
    double cargoArea = pixy.getArea(cargo);

    // if no ball is detected, rotate towards the edge from which the ball disappears
    if (cargo == null) {
      if (lastXOffset < 155){
        drivetrain.drive(new ChassisSpeeds(0, 0, 1));
      }
      else {
        drivetrain.drive(new ChassisSpeeds(0, 0, -1));
      }
    }
    // rotate to the largest ball seen by the pixy camera
    else {
      double cargoXOffset = cargo.getX();
      lastXOffset = cargoXOffset;

      double rotationSpeed = rotationPidController.calculate(cargoXOffset);
      double translationXSpeed = translationXPidController.calculate(cargoArea);

      drivetrain.drive(new ChassisSpeeds(translationXSpeed, 0, rotationSpeed));
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds(0, 0, 0));
    pixy.turnOffLights();
  }

  @Override
  public boolean isFinished() {
    return (Math.abs(rotationSpeed) < 0.01 && Math.abs(translationXSpeed) < 0.01) ||
          (Math.abs(currentXOffset - 155) < 3);
  }
}