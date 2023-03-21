// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeedMode;



public class AutoIntakeCommand extends CommandBase {
  public enum IntakeMode {
    extended,
    retracted
  };
  
  private IntakeSubsystem intake;
  private FeederSubsystem feeder;
  private IntakeMode currentMode;
  
  public AutoIntakeCommand(IntakeSubsystem intake, FeederSubsystem feeder, IntakeMode mode) {
    this.intake = intake;
    this.feeder = feeder;
    currentMode = mode;
    addRequirements(intake, feeder);
  }

  @Override
  public void initialize() {
    if(currentMode == IntakeMode.extended){
      intake.extend();
      feeder.setFeedMode(FeedMode.INTAKE);
    }
    if(currentMode == IntakeMode.retracted){
      intake.retract();
      feeder.setFeedMode(FeedMode.STOPPED);
    }
  }

  @Override
  public void execute() {
    if(currentMode == IntakeMode.extended){
      intake.spinBallIn();
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}