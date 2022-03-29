package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.HookPosition;

public class SwitchRedHookCommand extends CommandBase {

	private ClimberSubsystem climber;
	private HookPosition hookPosition;

	public SwitchRedHookCommand(ClimberSubsystem climber, HookPosition hookPosition) {
		this.climber = climber;
		this.hookPosition = hookPosition;
	}

	@Override
  	public void initialize() {
    	climber.setRedHookPosition(hookPosition);
  	}

	@Override
	public void execute() {
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return climber.isRedHookAtPosition();
	}
}
