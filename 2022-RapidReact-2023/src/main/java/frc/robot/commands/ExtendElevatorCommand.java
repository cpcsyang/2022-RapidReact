package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ExtendElevatorCommand extends CommandBase {

	private ClimberSubsystem climber;
	private double speed;

	public ExtendElevatorCommand(ClimberSubsystem climber, double speed) {
		this.climber = climber;
		this.speed = speed;

		addRequirements(climber);
	}

	@Override
	public void initialize() {
		climber.elevator.extendElevator(speed);
	}

	@Override
	public void end(boolean interrupted) {
		climber.elevator.extendElevator(0);
		climber.windmill.setHomeStatus(false);
	}
}
