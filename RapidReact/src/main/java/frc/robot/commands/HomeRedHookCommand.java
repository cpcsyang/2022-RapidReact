// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class HomeRedHookCommand extends CommandBase {
  
  private ClimberSubsystem climber;
  private Timer hookTimer;
  private double homeSpeed = 0.05;

  public HomeRedHookCommand(ClimberSubsystem climber) {
    this.climber = climber;
    hookTimer = new Timer();
    
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    hookTimer.stop();
    hookTimer.start();
    climber.hookRed.setSpeed(homeSpeed);
  }

  @Override
  public void end(boolean interupted){
    climber.hookRed.zeroEncoder();
    climber.hookRed.setTargetAngle(0);
  }

  @Override
  public boolean isFinished() {
    return (Math.abs(climber.hookRed.getVelocity()) < 0.01 && hookTimer.get() > 0.25);
  }
}
