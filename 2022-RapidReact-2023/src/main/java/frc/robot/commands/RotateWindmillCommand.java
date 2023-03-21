// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.WindmillState;

public class RotateWindmillCommand extends CommandBase {

	private ClimberSubsystem climber;
	private WindmillState position;

	public RotateWindmillCommand(ClimberSubsystem climber, WindmillState position) {
		this.climber = climber;
		this.position = position;
		addRequirements(climber);
	}

	@Override
	public void initialize() {
		climber.windmill.rotateWindmill(position);
	}

	@Override
	public boolean isFinished() {
		return climber.windmill.isAtTargetPosition();
	}
}
