package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.AutoFollowCargoCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TeleOpDriveCommand;
import frc.robot.drivers.Pigeon;
import frc.robot.drivers.Pixy;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.utils.ControllerUtils;
import frc.robot.utils.Path;
import frc.robot.utils.Ranger;
import frc.robot.utils.SimpleRanger;
import frc.robot.utils.SwerveDriveConfig;
import frc.robot.utils.SwerveModuleConfig;
import frc.robot.utils.Path.PATH;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.*;

import java.util.List;

/**
 * This class is where the bulk of the robot should be declared.
 */
public class RobotContainer {

    private final XboxController driveController = new XboxController(Constants.DRIVE_CONTROLLER_PORT);
    @SuppressWarnings("unused")
    private final XboxController operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);

    Pigeon pigeon = new Pigeon(DRIVETRAIN_PIGEON_ID);
    private final Pixy pixy = new Pixy(Pixy.TEAM_RED);
    private final Ranger ranger = new SimpleRanger();

    // Subsystems
    private FeederSubsystem feeder;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private DrivetrainSubsystem drivetrain;
    private LimelightSubsystem limelight;

    // Commands
    private TeleOpDriveCommand teleOpDrive;
    private AutoShootCommand shoot;
    private AutoDriveCommand driveBack;
    private SwerveControllerCommand followTrajectory;
    private AutoFollowCargoCommand followCargo;

    public RobotContainer() {

        createSubsystems(); // Create our subsystems.
        createCommands(); // Create our commands
        configureButtonBindings(); // Setup our button bindings

        drivetrain.setDefaultCommand(teleOpDrive);
    }

    /**
     * Create all of our robot's subsystem objects here.
     */
    void createSubsystems() {
        //intake = new IntakeSubsystem(INTAKE_MOTOR_CAN_ID, INTAKE_RETRACT_SOLENOID_CHAN, INTAKE_EXTEND_SOLENOID_CHAN);
        feeder = new FeederSubsystem(FEEDER_MOTOR_CAN_ID, FEEDER_ENTRY_SENSOR_DIO, FEEDER_EXIT_SENSOR_DIO);
        shooter = new ShooterSubsystem(SHOOTER_MOTOR_1_CAN_ID, SHOOTER_MOTOR_2_CAN_ID, HOOD_MOTOR_CAN_ID,
                HOOD_LIMITSWITCH_DIO, ranger);
        intake = new IntakeSubsystem(INTAKE_MOTOR_CAN_ID, BOOM_RETRACT_SOLENOID_CHAN, BOOM_EXTEND_SOLENOID_CHAN,
                ARM_RETRACT_SOLENOID_CHAN, ARM_EXTEND_SOLENOID_CHAN);

        // Setup our server drivetrain subsystem
        SwerveModuleConfig fl = new SwerveModuleConfig(FRONT_LEFT_MODULE_DRIVE_MOTOR, FRONT_LEFT_MODULE_STEER_MOTOR,
                FRONT_LEFT_MODULE_STEER_ENCODER, FRONT_LEFT_MODULE_STEER_OFFSET);
        SwerveModuleConfig fr = new SwerveModuleConfig(FRONT_RIGHT_MODULE_DRIVE_MOTOR, FRONT_RIGHT_MODULE_STEER_MOTOR,
                FRONT_RIGHT_MODULE_STEER_ENCODER, FRONT_RIGHT_MODULE_STEER_OFFSET);
        SwerveModuleConfig bl = new SwerveModuleConfig(BACK_LEFT_MODULE_DRIVE_MOTOR, BACK_LEFT_MODULE_STEER_MOTOR,
                BACK_LEFT_MODULE_STEER_ENCODER, BACK_LEFT_MODULE_STEER_OFFSET);
        SwerveModuleConfig br = new SwerveModuleConfig(BACK_RIGHT_MODULE_DRIVE_MOTOR, BACK_RIGHT_MODULE_STEER_MOTOR,
                BACK_RIGHT_MODULE_STEER_ENCODER, BACK_RIGHT_MODULE_STEER_OFFSET);
        SwerveDriveConfig swerveConfig = new SwerveDriveConfig(fl, fr, bl, br, DRIVETRAIN_TRACKWIDTH_METERS,
                DRIVETRAIN_WHEELBASE_METERS, DRIVE_TRAIN_WHEEL_DIAMETER_METERS);
        drivetrain = new DrivetrainSubsystem(swerveConfig, pigeon, pixy);

        limelight = new LimelightSubsystem(36, 0.5842, 2.6414);
    }

    /**
     * Create all of our robot's command objects here.
     */
    void createCommands() {
        teleOpDrive = new TeleOpDriveCommand(
                drivetrain,
                () -> -ControllerUtils.modifyAxis(driveController.getLeftY()) * drivetrain.maxVelocity,
                () -> -ControllerUtils.modifyAxis(driveController.getLeftX()) * drivetrain.maxVelocity,
                () -> -ControllerUtils.modifyAxis(driveController.getRightX()) * drivetrain.maxAngularVelocity);
    }

    /**
     * Use this method to define your button->command mappings.
     */
    private void configureButtonBindings() {

        // Reset the gyroscope on the Pigeon.
        new JoystickButton(driveController, Button.kStart.value)
                .whenPressed(new InstantCommand(() -> drivetrain.resetGyroscope()));

        // Schedule the Shoot command to fire a cargo       
        new JoystickButton(driveController, Button.kY.value).
                whenHeld(new ShootCommand(shooter, feeder, limelight, (() -> driveController.getRightTriggerAxis() > 0.8)));

        // Schedule the Intake command to pick-up cargo        
        new JoystickButton(driveController, Button.kRightBumper.value).
                whenHeld(new IntakeCommand(intake, feeder, (() -> driveController.getLeftBumper())));  
        
        // Temporary test commands to be removed before competition
        new JoystickButton(driveController, Button.kA.value)
               .whenPressed(getAutoDriveCommand());
        new JoystickButton(driveController, Button.kB.value)
               .whenPressed(getFollowCargoCommand());
        
        //new POVButton(driveController, 0).whenPressed(new InstantCommand(() -> shooter.setAngle(67.0), shooter));      
    }

    /**
     * The main {@link Robot} class calls this to get the command to run during
     * autonomous.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
            return null;
            /*
            return new SequentialCommandGroup(getAutoShootCommand(),
                                                getAutoDriveCommand(),
                                                getFollowTrajectoryCommand(), // TODO need an intake command
                                                getAutoShootCommand());
        */
    }

    public Command getFollowTrajectoryCommand() {
        Path path = new Path(PATH.backOutOfTarmac);
        followTrajectory = new SwerveControllerCommand(path.getTrajectory(),
                                                        drivetrain::getPose,
                                                        drivetrain.getKinematics(),
                                                        path.getPidController(),
                                                        path.getPidController(),
                                                        path.getAnglePidController(),
                                                        drivetrain::setModuleStates,
                                                        drivetrain);
        return followTrajectory;
    }

    public Command getFollowCargoCommand() {
        followCargo = new AutoFollowCargoCommand(drivetrain, pixy);
        return followCargo;
    }

    public Command getAutoDriveCommand() {
        driveBack = new AutoDriveCommand(drivetrain, new Translation2d(-3, 0), Rotation2d.fromDegrees(-90));
        return driveBack;
    }

    public Command getAutoShootCommand() {
            shoot = new AutoShootCommand(shooter, feeder, limelight);
            return shoot;
    }
}
