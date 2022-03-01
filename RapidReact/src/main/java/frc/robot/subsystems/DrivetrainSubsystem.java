package frc.robot.subsystems;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.drivers.Pigeon;
import frc.robot.drivers.Pixy;
import frc.robot.utils.SwerveDriveConfig;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

public class DrivetrainSubsystem extends SubsystemBase {

    public static final double MAX_VOLTAGE = 12.0;

    public final double trackWidth;
    public final double wheelbase;
    public final double wheelDiameter;
    public final double maxVelocity;
    public final double maxAngularVelocity;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private final Pigeon pigeon;
    private final Pixy pixy;

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private SwerveModuleState[] states;
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private NetworkTableEntry poseXEntry;
    private NetworkTableEntry poseYEntry;
    private NetworkTableEntry poseAbsoluteAngleEntry;
    private NetworkTableEntry driveSignalXEntry;
    private NetworkTableEntry driveSignalYEntry;
    private NetworkTableEntry driveSignalRotationEntry;
    private NetworkTableEntry cargoAreaEntry;
    private NetworkTableEntry cargoXEntry;

    public DrivetrainSubsystem(SwerveDriveConfig config, Pigeon pigeon, Pixy pixy) {

        this.pigeon = pigeon;
        this.pixy = pixy;

        // Physical constants for this drive base.
        trackWidth = config.trackWidth;
        wheelbase = config.wheelbase;
        wheelDiameter = config.wheelDiameter;

        // Maximum module velocity in meters/second
        maxVelocity = 6380.0 / 60.0 * SdsModuleConfigurations.MK4_L4.getDriveReduction() * wheelDiameter
                * Math.PI;

        // Maximum angular velocity in radians/second.
        maxAngularVelocity = maxVelocity / Math.hypot(trackWidth / 2.0, wheelbase / 2.0);

        // Create the kinematics object used to calculate desired module states.
        kinematics = new SwerveDriveKinematics(
                new Translation2d(trackWidth / 2.0, wheelbase / 2.0), // FL
                new Translation2d(trackWidth / 2.0, -wheelbase / 2.0), // FR
                new Translation2d(-trackWidth / 2.0, wheelbase / 2.0), // BL
                new Translation2d(-trackWidth / 2.0, -wheelbase / 2.0) // BR
        );

        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), new Pose2d());

        // Create our swerve module objects.
        ShuffleboardTab drivetrainModuletab = Shuffleboard.getTab("Swerve Modules");
        frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                drivetrainModuletab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                Mk4SwerveModuleHelper.GearRatio.L4,
                config.frontLeft.driveMotorCanId,
                config.frontLeft.steerMotorCanId,
                config.frontLeft.encoderCanId,
                config.frontLeft.encoderOffset);

        frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                drivetrainModuletab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0),
                Mk4SwerveModuleHelper.GearRatio.L4,
                config.frontRight.driveMotorCanId,
                config.frontRight.steerMotorCanId,
                config.frontRight.encoderCanId,
                config.frontRight.encoderOffset);

        backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                drivetrainModuletab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0),
                Mk4SwerveModuleHelper.GearRatio.L4,
                config.backLeft.driveMotorCanId,
                config.backLeft.steerMotorCanId,
                config.backLeft.encoderCanId,
                config.backLeft.encoderOffset);

        backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                drivetrainModuletab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0),
                Mk4SwerveModuleHelper.GearRatio.L4,
                config.backRight.driveMotorCanId,
                config.backRight.steerMotorCanId,
                config.backRight.encoderCanId,
                config.backRight.encoderOffset);

        ShuffleboardTab drivetrainRobotTab = Shuffleboard.getTab("Drivetrain");
        poseXEntry = drivetrainRobotTab.add("Pose X", 0.0)
                .withPosition(0, 0)
                .withSize(1, 1)
                .getEntry();
        poseYEntry = drivetrainRobotTab.add("Pose Y", 0.0)
                .withPosition(0, 1)
                .withSize(1, 1)
                .getEntry();
        poseAbsoluteAngleEntry = drivetrainRobotTab.add("Pose Angle", 0.0)
                .withPosition(0, 2)
                .withSize(1, 1)
                .getEntry();
        ShuffleboardLayout driveSignalContainer = drivetrainRobotTab
                .getLayout("Drive Signal", BuiltInLayouts.kGrid)
                .withPosition(0, 3)
                .withSize(3, 1);
        driveSignalYEntry = driveSignalContainer.add("Drive Signal Strafe", 0.0).getEntry();
        driveSignalXEntry = driveSignalContainer.add("Drive Signal Forward", 0.0).getEntry();
        driveSignalRotationEntry = driveSignalContainer.add("Drive Signal Rotation", 0.0).getEntry();
    }

    public void resetPosition() {
        odometry.resetPosition(new Pose2d(), pigeon.getRotation2d());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetGyroscope() {
        pigeon.reset();
    }

    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(pigeon.getAngle());
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }

    public void setModuleStates() {
        states = kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxVelocity);

        frontLeftModule.set(states[0].speedMetersPerSecond / maxVelocity * MAX_VOLTAGE,
                states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond / maxVelocity * MAX_VOLTAGE,
                states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond / maxVelocity * MAX_VOLTAGE,
                states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond / maxVelocity * MAX_VOLTAGE,
                states[3].angle.getRadians());
    }

    @Override
    public void periodic() {
        setModuleStates();
        odometry.updateWithTime(Timer.getFPGATimestamp(), getGyroscopeRotation(), states);

        driveSignalYEntry.setDouble(chassisSpeeds.vyMetersPerSecond);
        driveSignalXEntry.setDouble(chassisSpeeds.vxMetersPerSecond);
        driveSignalRotationEntry.setDouble(chassisSpeeds.omegaRadiansPerSecond);
        poseXEntry.setDouble(getPose().getTranslation().getX());
        poseYEntry.setDouble(getPose().getTranslation().getY());
        poseAbsoluteAngleEntry.setDouble(getPose().getRotation().getDegrees());

        Block cargo = pixy.getLargestBlock();
        cargoAreaEntry.setDouble(pixy.getArea(cargo));
        cargoXEntry.setDouble(pixy.getX(cargo));

        // pose angle entry (for trajectory following tuning)
    }
}