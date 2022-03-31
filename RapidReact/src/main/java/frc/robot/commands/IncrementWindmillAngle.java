// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class IncrementWindmillAngle extends CommandBase {
  /** Creates a new IncrementWindmillAngle. */
  private ClimberSubsystem climber;
  private double increment;

  public IncrementWindmillAngle(ClimberSubsystem climber, double increment) {
    this.climber = climber;
    this.increment = increment;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.windmill.setAngle(climber.windmill.getTargetAngle() + increment);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
