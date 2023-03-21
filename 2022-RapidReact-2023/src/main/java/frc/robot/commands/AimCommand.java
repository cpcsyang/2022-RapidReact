// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AimCommand extends CommandBase {
	private DrivetrainSubsystem drivetrain;
	private LimelightSubsystem limelight;

	private PIDController tController = new PIDController(0.055, 0, 0.005); // 0.055, 0, 0.005

	private double currentOffset;
	private double rotationSpeed;

	private DoubleSupplier translationXSupplier;
	private DoubleSupplier translationYSupplier;
	
    private double staticConst = 0.2; 
		// set pid to 0, increase until slightly rotating

	// during tele
	public AimCommand(LimelightSubsystem limelight, DrivetrainSubsystem drivetrain,
			DoubleSupplier translationXSupplier,
			DoubleSupplier translationYSupplier) {
		this.limelight = limelight;
		this.drivetrain = drivetrain;

		this.translationXSupplier = translationXSupplier;
		this.translationYSupplier = translationYSupplier;

		tController.setSetpoint(0);
		tController.setTolerance(3);
		limelight.setTolerance(3);

		addRequirements(drivetrain);
	}

	// during auto
	public AimCommand(LimelightSubsystem limelight, DrivetrainSubsystem drivetrain) {
		this(limelight, drivetrain, () -> 0, () -> 0);
	}

	@Override
	public void execute() {
		// calculate rotation speed
		currentOffset = limelight.getXOffset();
		rotationSpeed = tController.calculate(currentOffset) 
						+ Math.copySign(staticConst, tController.getPositionError());

		// create drive signal
		drivetrain.drive(
			ChassisSpeeds.fromFieldRelativeSpeeds(
				translationXSupplier.getAsDouble(),
				translationYSupplier.getAsDouble(),
				rotationSpeed,
				drivetrain.getPose().getRotation()));
	}

	@Override
	public void end(boolean interrupted) {
		// delete drive signal
		drivetrain.drive(new ChassisSpeeds(0, 0, 0));
	}
}
