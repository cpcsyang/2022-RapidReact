package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.filter.SlewRateLimiter;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private static final double MAX_LINEAR_ACCELERATION = 15.0;
    private static final double MAX_ANGULAR_ACCELERATION = Math.toDegrees(2.5);

    private final DrivetrainSubsystem drivetrainSubsystem;

    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;

    private final SlewRateLimiter xRateLimiter = new SlewRateLimiter(MAX_LINEAR_ACCELERATION);
    private final SlewRateLimiter yRateLimiter = new SlewRateLimiter(MAX_LINEAR_ACCELERATION);
    private final SlewRateLimiter rotationRateLimiter = new SlewRateLimiter(MAX_ANGULAR_ACCELERATION);

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        //drivetrainSubsystem.resetPosition();
	
	xRateLimiter.reset(translationXSupplier.getAsDouble());
	yRateLimiter.reset(translationYSupplier.getAsDouble());
	rotationRateLimiter.reset(rotationSupplier.getAsDouble());
    }

    @Override
    public void execute() {
        drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        xRateLimiter.calculate(translationXSupplier.getAsDouble()),
                        yRateLimiter.calculate(translationYSupplier.getAsDouble()),
                        rotationRateLimiter.calculate(rotationSupplier.getAsDouble()),
                        drivetrainSubsystem.getPose().getRotation()
                ));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
