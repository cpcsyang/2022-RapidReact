package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import frc.robot.utils.FiringSolution;
import frc.robot.utils.MathUtils;
import frc.robot.utils.Ranger;

public class ShooterSubsystem extends SubsystemBase {

    private enum MotorState {
        STOPPED,
        IDLE, // running at a constant rpm
        SHOOTING, // rpm changed by command
        STOPPING
    }

    // Subsystem Constants
    private static final int IDLE_CURRENT = 10; 
    private static final int MAX_CURRENT = 60;
    private double highestCurrent = 0;

    private static final double MAX_RPM = 6000;
    private double IDLE_RPM;
    private double IDLE_ANGLE;
    private static final double shooterBeltRatio = 0.66;
    private static final double speedIncrement = 100;
    private static final double speedPercentTolerance = 0.04;
    private static final double speedTolerance = 75;
    private static final double speedFirstTolerance = 0.02;
    private static final double speedSecondTolerance = 0.04;

    private static final double MAX_HOOD_ANGLE = 85;
    private static final double MIN_HOOD_ANGLE = 67;
    private static final double HOOD_LOWER_LIMIT = 0;
    private static final double HOOD_UPPER_LIMIT = 13.5;
    private static final double ROTATIONS_PER_DEGREE = (HOOD_UPPER_LIMIT - HOOD_LOWER_LIMIT)
            / (MAX_HOOD_ANGLE - MIN_HOOD_ANGLE);
    private static final double angleIncrement = 1;

    // Shooter PID coefficients constants
    private static final double kShooterP = 0.0001;
    private static final double kShooterI = 0.000000;
    private static final double kShooterD = 0.000000;
    private static final double kShooterIz = 0.000000;
    private static final double kShooterFF = 0.000290;
    private static final double kShooterMaxOutput = 1.000000;
    private static final double kShooterMinOutput = 0.000000;


    // Hood PID coefficients
    private static final double kHoodP = 0.1;
    private static final double kHoodI = 1e-4;
    private static final double kHoodD = 1;
    private static final double kHoodIz = 0;
    private static final double kHoodFF = 0;
    private static final double kHoodMaxOutput = 1;
    private static final double kHoodMinOutput = -1;

    private Ranger ranger;
    private CANSparkMax shooterMotor1;
    private CANSparkMax shooterMotor2;
    private RelativeEncoder shooterEncoder;
    private SparkMaxPIDController shooterPidController;

    private CANSparkMax hoodMotor;
    private RelativeEncoder hoodEncoder;
    private SparkMaxPIDController hoodPidController;
    private DigitalInput hoodLimit;

    private MotorState motorState = MotorState.STOPPED;
    private double currentSpeed = 0;
    private double targetSpeed = 0;
    private double speedAdjust = 0; // TODO speed adjust
    private double speedError;
    private double speedErrorPercent;

    private boolean parkingHood = true;
    private double currentAngle = 0;
    private double targetAngle = 0;

    private double currentRange = 0.0;
    private double currentXOffset = 0;

    public boolean aligned = false;
    public boolean atSpeed = false;

    private Timer hoodTimer;

    private NetworkTableEntry currentSpeedEntry;
    private NetworkTableEntry targetSpeedEntry;
    private NetworkTableEntry speedAdjustEntry;   
    private NetworkTableEntry shooterEncoderEntry;
    private NetworkTableEntry speedErrorEntry;
    private NetworkTableEntry speedErrorPercentEntry;
    private NetworkTableEntry currentAngleEntry;
    private NetworkTableEntry targetAngleEntry;
    private NetworkTableEntry hoodEncoderEntry;
    private NetworkTableEntry hoodLimitSwitchEntry;
    private NetworkTableEntry readyToShootEntry;
    private NetworkTableEntry currentRangeEntry;
    private NetworkTableEntry currentXEntry;
    private NetworkTableEntry highestCurrentEntry;
    private NetworkTableEntry alignedWithHubEntry;
    private NetworkTableEntry shooterMotorCurrentEntry;
    private NetworkTableEntry hoodMotorCurrentEntry;
 
    /** Creates a new instance of the Shooter subsystem. */
    public ShooterSubsystem(int shooterMotor1CANID, int shooterMotor2CANID, int hoodMotorCANID, int hoodLimitDio,
            Ranger ranger) {

        hoodTimer = new Timer();

        this.ranger = ranger;
        IDLE_RPM = ranger.getFiringSolution("hub").speed;
        IDLE_ANGLE = ranger.getFiringSolution("hub").angle;

        shooterMotor1 = new CANSparkMax(shooterMotor1CANID, MotorType.kBrushless);
        shooterMotor1.setInverted(true);
        shooterMotor1.setIdleMode(IdleMode.kCoast);
        shooterEncoder = shooterMotor1.getEncoder();
        shooterEncoder.setVelocityConversionFactor(shooterBeltRatio);

        shooterMotor2 = new CANSparkMax(shooterMotor2CANID, MotorType.kBrushless);
        shooterMotor2.setIdleMode(IdleMode.kCoast);
        shooterMotor2.follow(shooterMotor1, true);
        shooterMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
        shooterMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        shooterMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
        shooterMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);

        hoodMotor = new CANSparkMax(hoodMotorCANID, MotorType.kBrushless);
        hoodMotor.setIdleMode(IdleMode.kBrake);
        hoodMotor.setSmartCurrentLimit(40);
        hoodEncoder = hoodMotor.getEncoder();
        hoodLimit = new DigitalInput(hoodLimitDio);

        hoodTimer = new Timer();

        shooterPidController = shooterMotor1.getPIDController();
        shooterPidController.setP(kShooterP);
        shooterPidController.setI(kShooterI);
        shooterPidController.setD(kShooterD);
        shooterPidController.setIZone(kShooterIz);
        shooterPidController.setFF(kShooterFF);
        shooterPidController.setOutputRange(kShooterMinOutput, kShooterMaxOutput);

        hoodPidController = hoodMotor.getPIDController();
        hoodPidController.setP(kHoodP);
        hoodPidController.setI(kHoodI);
        hoodPidController.setD(kHoodD);
        hoodPidController.setIZone(kHoodIz);
        hoodPidController.setFF(kHoodFF);
        hoodPidController.setOutputRange(kHoodMinOutput, kHoodMaxOutput);

        initTelemetry();
    }

    @Override
    public void periodic() {
        currentSpeed = shooterEncoder.getVelocity();
        speedError = currentSpeed - targetSpeed;
        speedErrorPercent = targetSpeed > 0 ? speedError / targetSpeed : 0;

        currentAngle = encoderPositionToAngle(hoodEncoder.getPosition());


        if(DriverStation.isEnabled()){
            parkHood();
        }

        // If we are in the process of stopping the shooter motors then slowly ramp the
        // target speed down so the momentum of the flywheels doesn't damage the drive belts.
        if (motorState == MotorState.STOPPING) {
            setSpeed(targetSpeed - 20);
        }

        // Read the current speed adjustment value from network table.
        // Value is set in shuffleboard.
        speedAdjust = speedAdjustEntry.getDouble(speedAdjust);

        updateTelemetry();
    }

    // The shooter is not avaialble for use until after it finishes parking the hood
    // this method lets you find you find out if that has completed.
    public boolean available() {
        return !parkingHood;
    }

    public boolean ready() {
        /*
        if (!atSpeed) {
            atSpeed = withinFirstTolerance();
        }
        else {
            atSpeed = withinSecondTolerance();
        }

        return atSpeed;
        */
        return atTargetSpeed();
    }

    // ---------------------------------------------------------------------------
    // Shooter control methods
    // ---------------------------------------------------------------------------

    public void idle() {
        motorState = MotorState.IDLE;

        setSpeed(IDLE_RPM);
        setAngle(IDLE_ANGLE);

        shooterMotor1.setSmartCurrentLimit(IDLE_CURRENT);
        shooterMotor2.setSmartCurrentLimit(IDLE_CURRENT);
    }

    public void shoot() {
      motorState = MotorState.SHOOTING;

      setSpeed(targetSpeed);

      shooterMotor1.setSmartCurrentLimit(MAX_CURRENT);
      shooterMotor2.setSmartCurrentLimit(MAX_CURRENT);
    }

    public void stop() {
        motorState = MotorState.STOPPING;
    }

    public void setRange(double range) {
        currentRange = range;

        FiringSolution solution = ranger.getFiringSolution(currentRange);
        setSpeed(solution.speed);
        setAngle(solution.angle);
    }

    public void setRange(String range) {
        FiringSolution solution = ranger.getFiringSolution(range);
        setSpeed(solution.speed);
        setAngle(solution.angle);
    }

    public void setSpeed(double newTarget) {
        // Set new target speed to value provided by caller plus the current speed adjust
        // set in shuffleboard.
        targetSpeed = newTarget + speedAdjust;

        if (targetSpeed > MAX_RPM) {
            targetSpeed = MAX_RPM;
        } else if (targetSpeed < 0) {
            targetSpeed = 0;
            motorState = MotorState.STOPPED;
        }

        shooterPidController.setReference(targetSpeed, ControlType.kVelocity);
    }

    public void increaseSpeed() {
        targetSpeed += speedIncrement;

        if (motorState == MotorState.SHOOTING) {
            setSpeed(targetSpeed);
        }
    }

    public void decreaseSpeed() {
        targetSpeed -= speedIncrement;

        if (motorState == MotorState.SHOOTING) {
            setSpeed(targetSpeed);
        }
    }

    private boolean atTargetSpeed() {
        return MathUtils.WithinDelta(currentSpeed, targetSpeed, speedTolerance);
    }

    private boolean withinFirstTolerance() {
        double delta = currentSpeed * speedFirstTolerance;
        return MathUtils.WithinDelta(currentSpeed, targetSpeed, delta);
    }

    private boolean withinSecondTolerance() {
        double delta = currentSpeed * speedSecondTolerance;
        return MathUtils.WithinDelta(currentSpeed, targetSpeed, delta);
    }

    // ---------------------------------------------------------------------------
    // Hood Control methods
    // ---------------------------------------------------------------------------

    public void setAngle(double angle) {

        targetAngle = angle;

        if (targetAngle > MAX_HOOD_ANGLE) {
            targetAngle = MAX_HOOD_ANGLE;
        } else if (targetAngle < MIN_HOOD_ANGLE) {
            targetAngle = MIN_HOOD_ANGLE;
        }

        hoodPidController.setReference(angleToEncoderPosition(targetAngle), ControlType.kPosition);
    }

    public void raiseHood() {
        setAngle(targetAngle - angleIncrement);
    }

    public void lowerHood() {
        setAngle(targetAngle + angleIncrement);
    }

    private double angleToEncoderPosition(double angle) {
        return (MAX_HOOD_ANGLE - angle) * ROTATIONS_PER_DEGREE;
    }

    private double encoderPositionToAngle(double position) {
        return ((HOOD_UPPER_LIMIT - position) / ROTATIONS_PER_DEGREE) + MIN_HOOD_ANGLE;
    }

    /**
     * Lower the hood until it trips the limit switch and then reset the encoder to
     * establish our zero position.  Called by periodic.
     */
    private void parkHood() {

        hoodTimer.start();
        // If we are not in the process of parking the hood then just bail out now.
        // parkingHood is initialized to true so we will always fall through this
        // check on the first call.
        if (!parkingHood) {
            return;
        }

        
        // if the hood is not at the limit then set power to start lowering it and return.
        if (Math.abs(hoodEncoder.getVelocity()) > 100 || hoodTimer.get() < 0.25) {
            hoodMotor.set(-0.1);
            return;
        }

        // Hood has reached limit, clear parking flag, stop motor and zero encoder.
        
        parkingHood = false;
        hoodTimer.stop();
        hoodMotor.set(0);
        hoodEncoder.setPosition(-0.25);
        setAngle(MAX_HOOD_ANGLE);
    }

    // ---------------------------------------------------------------------------
    // Telemetry
    // ---------------------------------------------------------------------------

    private void initTelemetry() {
        ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
        ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");

        // Shooter Data
        currentSpeedEntry = shooterTab.add("Current Speed", 0)
                .withPosition(0, 2)
                .withSize(1, 1)
                .getEntry();

        targetSpeedEntry = shooterTab.add("Target Speed", 0)
                .withPosition(1, 2)
                .withSize(1, 1)
                .getEntry();
        
        speedAdjustEntry = driverTab.add("Shooter Speed Adjust", speedAdjust)
                .withPosition(0, 1)
                .withSize(4, 1)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", -250, "max", 250, "block increment", 25))
                .getEntry();

        speedErrorEntry = shooterTab.add("Error", 0)
                .withPosition(2, 2)
                .withSize(1, 1)
                .getEntry();

        speedErrorPercentEntry = shooterTab.add("Error %", 0)
                .withPosition(3, 2)
                .withSize(1, 1)
                .getEntry();

        shooterEncoderEntry = shooterTab.add("Encoder", 0)
                .withPosition(4, 2)
                .withSize(1, 1)
                .getEntry();

        currentRangeEntry = shooterTab.add("Distance", 0)
                .withPosition(5, 2)
                .withSize(1, 1)
                .getEntry();
        currentXEntry = shooterTab.add("X offset", 0)
                .withPosition(5, 1)
                .withSize(1, 1)
                .getEntry();

        readyToShootEntry = shooterTab.add("Ready", false)
                .withPosition(6, 2)
                .withSize(1, 1)
                .getEntry();

        alignedWithHubEntry = shooterTab.add("Aligned", false)
                .withPosition(7, 2)
                .withSize(1, 1)
                .getEntry();

        // Hood Data
        currentAngleEntry = shooterTab.add("Current Angle", 0)
                .withPosition(0, 3)
                .withSize(1, 1)
                .getEntry();

        targetAngleEntry = shooterTab.add("Target Angle", 0)
                .withPosition(1, 3)
                .withSize(1, 1)
                .getEntry();

        hoodEncoderEntry = shooterTab.add("Hood Encoder", 0)
                .withPosition(2, 3)
                .withSize(1, 1)
                .getEntry();

        hoodLimitSwitchEntry = shooterTab.add("Hood Limit", false)
                .withPosition(3, 3)
                .withSize(1, 1)
                .getEntry();

        // current data
        highestCurrentEntry = shooterTab.add("Highest Current", 0)
                .withPosition(7, 0)
                .withSize(1, 1)
                .getEntry();

        shooterMotorCurrentEntry = shooterTab.add("shooter motor current", 0)
                .withPosition(8, 0)
                .withSize(1, 1)
                .getEntry();
        hoodMotorCurrentEntry = shooterTab.add("hood motor current", 0)
                .withPosition(9, 0)
                .withSize(1, 1)
                .getEntry();
    }

    private void updateTelemetry() {
        currentSpeedEntry.setNumber(currentSpeed);
        targetSpeedEntry.setNumber(targetSpeed);
        speedErrorEntry.setNumber(speedError);
        speedErrorPercentEntry.setNumber(speedErrorPercent);
        shooterEncoderEntry.setNumber(shooterEncoder.getPosition());
        currentRangeEntry.setNumber(currentRange);
        currentXEntry.setNumber(currentXOffset);
        readyToShootEntry.forceSetBoolean(ready());
        alignedWithHubEntry.setBoolean(aligned);

        currentAngleEntry.setNumber(currentAngle);
        targetAngleEntry.setNumber(targetAngle);
        hoodEncoderEntry.setNumber(hoodEncoder.getPosition());
        hoodLimitSwitchEntry.forceSetBoolean(hoodLimit.get());

        highestCurrentEntry.setNumber(getHighestCurrent());
        shooterMotorCurrentEntry.setValue(shooterMotor1.getOutputCurrent());
        hoodMotorCurrentEntry.setValue(hoodMotor.getOutputCurrent());
    }

    private double getHighestCurrent() {
      double current = hoodMotor.getOutputCurrent();
      if (current > highestCurrent) {
        highestCurrent = current;
      }
      return highestCurrent;
    }
}
