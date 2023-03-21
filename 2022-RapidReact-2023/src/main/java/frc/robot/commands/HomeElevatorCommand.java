// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class HomeElevatorCommand extends CommandBase {
  /** Creates a new HomeElevatorCommand. */
  private ClimberSubsystem climber;
  private Timer timer;
  
  public HomeElevatorCommand(ClimberSubsystem climber) {
    this.climber = climber;
    timer = new Timer();

    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.stop();
    timer.start();
    climber.elevator.extendElevator(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.elevator.zeroEncoder();
    climber.elevator.setTargetHeight(0);
    climber.windmill.setHomeStatus(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(climber.elevator.getVelocity()) < 5.0 && timer.get() >= 0.25);
  }
}
