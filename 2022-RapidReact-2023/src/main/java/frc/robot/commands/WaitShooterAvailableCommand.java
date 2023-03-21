package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ShooterSubsystem;

public class WaitShooterAvailableCommand extends CommandBase {

    ShooterSubsystem shooter;

    public WaitShooterAvailableCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
    }

    // Command finishes when the shooter subsystem is ready to be used.
    @Override
    public boolean isFinished() {
        return shooter.available();
    }
}
