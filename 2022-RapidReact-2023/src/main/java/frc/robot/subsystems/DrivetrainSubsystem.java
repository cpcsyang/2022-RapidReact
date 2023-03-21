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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.drivers.Pigeon;
import frc.robot.utils.SwerveDriveConfig;

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
    // private final Pixy pixy;

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private Pose2d robotPosition;
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    private ChassisSpeeds velocity;

    private NetworkTableEntry poseXEntry;
    private NetworkTableEntry poseYEntry;
    private NetworkTableEntry poseAbsoluteAngleEntry;
    private NetworkTableEntry driveSignalXEntry;
    private NetworkTableEntry driveSignalYEntry;
    private NetworkTableEntry driveSignalRotationEntry;

 //   private NetworkTableEntry cargoAreaEntry;
 //   private NetworkTableEntry cargoXEntry;

    public DrivetrainSubsystem(SwerveDriveConfig config, Pigeon pigeon) { //Pixy pixy

        this.pigeon = pigeon;
        // this.pixy = pixy;

        // Physical constants for this drive base.
        trackWidth = config.trackWidth;
        wheelbase = config.wheelbase;
        wheelDiameter = config.wheelDiameter;

        // Maximum module velocity in meters/second
        maxVelocity = 6380.0 / 60.0 * SdsModuleConfigurations.MK4_L1.getDriveReduction() * wheelDiameter
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

        odometry = new SwerveDriveOdometry(kinematics, getGyroscopeRotation(), new Pose2d());

        // Create our swerve module objects.
        /*
        Front Left to Front Right
        Back Left to Front Left
        Front Right to Back Right
        Back Right to Back Left
        */
        ShuffleboardTab drivetrainModuletab = Shuffleboard.getTab("Swerve Modules");
        frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                drivetrainModuletab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                Mk4SwerveModuleHelper.GearRatio.L1,
                config.frontLeft.driveMotorCanId,
                config.frontLeft.steerMotorCanId,
                config.frontLeft.encoderCanId,
                config.frontLeft.encoderOffset);

        frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                drivetrainModuletab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0),
                Mk4SwerveModuleHelper.GearRatio.L1,
                config.frontRight.driveMotorCanId,
                config.frontRight.steerMotorCanId,
                config.frontRight.encoderCanId,
                config.frontRight.encoderOffset);

        backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                drivetrainModuletab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0),
                Mk4SwerveModuleHelper.GearRatio.L1,
                config.backLeft.driveMotorCanId,
                config.backLeft.steerMotorCanId,
                config.backLeft.encoderCanId,
                config.backLeft.encoderOffset);

        backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                drivetrainModuletab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0),
                Mk4SwerveModuleHelper.GearRatio.L1,
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
        // cargoAreaEntry = drivetrainRobotTab.add("Cargo Area", 0.0)
        //         .withPosition(1, 0)
        //         .withSize(1, 1)
        //         .getEntry();
        // cargoXEntry = drivetrainRobotTab.add("Cargo X", 0.0)
        //         .withPosition(1, 1)
        //         .withSize(1, 1)
        //         .getEntry();
        ShuffleboardLayout driveSignalContainer = drivetrainRobotTab
                .getLayout("Drive Signal", BuiltInLayouts.kGrid)
                .withPosition(0, 3)
                .withSize(3, 1);
        driveSignalYEntry = driveSignalContainer.add("Drive Signal Strafe", 0.0).getEntry();
        driveSignalXEntry = driveSignalContainer.add("Drive Signal Forward", 0.0).getEntry();
        driveSignalRotationEntry = driveSignalContainer.add("Drive Signal Rotation", 0.0).getEntry();

        //resetGyroscope();
    }

    public void resetPosition() {
        odometry.resetPosition(new Pose2d(), getGyroscopeRotation());
    }

    public void invertRotation() {
            odometry.resetPosition(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180)), pigeon.getRotation2d());
    }

    public void setAutoInitPose(Pose2d pose) {
            odometry.resetPosition(pose, getGyroscopeRotation());
    }

    public Pose2d getPose() {
        return robotPosition;
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

    public ChassisSpeeds getVelocity() {
            return velocity;
    }

    public double getMaxVelocity() {
            return maxVelocity;
    }

    public double getMaxAngularVelocity() {
            return maxAngularVelocity;
    }

    public SwerveDriveKinematics getKinematics() {
            return kinematics;
    }

    public void setModuleStates(SwerveModuleState[] states) {
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
                SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(states, maxVelocity);

                SwerveModuleState currentFrontLeft = new SwerveModuleState(frontLeftModule.getDriveVelocity(),
                                new Rotation2d(frontLeftModule.getSteerAngle()));
                SwerveModuleState currentFrontRight = new SwerveModuleState(frontRightModule.getDriveVelocity(),
                                new Rotation2d(frontRightModule.getSteerAngle()));
                SwerveModuleState currentBackLeft = new SwerveModuleState(backLeftModule.getDriveVelocity(),
                                new Rotation2d(backLeftModule.getSteerAngle()));
                SwerveModuleState currentBackRight = new SwerveModuleState(backRightModule.getDriveVelocity(),
                                new Rotation2d(backRightModule.getSteerAngle()));

                SwerveModuleState[] currentStates = new SwerveModuleState[4];
                currentStates[0] = currentFrontLeft;
                currentStates[1] = currentFrontRight;
                currentStates[2] = currentBackLeft;
                currentStates[3] = currentBackRight;

                robotPosition = odometry.update(getGyroscopeRotation(), currentStates);
                velocity = kinematics.toChassisSpeeds(currentStates);

                setModuleStates(states);

                driveSignalYEntry.setDouble(chassisSpeeds.vyMetersPerSecond);
                driveSignalXEntry.setDouble(chassisSpeeds.vxMetersPerSecond);
                driveSignalRotationEntry.setDouble(chassisSpeeds.omegaRadiansPerSecond);
                poseXEntry.setDouble(getPose().getTranslation().getX());
                poseYEntry.setDouble(getPose().getTranslation().getY());
                poseAbsoluteAngleEntry.setDouble(getPose().getRotation().getDegrees());

                // Block cargo = pixy.getLargestBlock();
                // cargoAreaEntry.setDouble(pixy.getArea(cargo));
                // cargoXEntry.setDouble(pixy.getX(cargo));

                // pose angle entry (for trajectory following tuning)
        }
}
