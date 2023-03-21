// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitForSecondsCommand extends CommandBase {

  private Timer timer;
  private double time;

  public WaitForSecondsCommand(double time) {
    this.time = time;
    timer = new Timer();
  } 

  @Override
  public void initialize() {
    timer.start();
  }

  @Override
  public boolean isFinished() {
    return timer.get() >= time;
  }
}
