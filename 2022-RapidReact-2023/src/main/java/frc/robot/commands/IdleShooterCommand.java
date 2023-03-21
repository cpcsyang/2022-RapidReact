package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class IdleShooterCommand extends CommandBase {
  ShooterSubsystem shooter;
  
  public IdleShooterCommand(ShooterSubsystem shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.idle();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    shooter.shoot();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
