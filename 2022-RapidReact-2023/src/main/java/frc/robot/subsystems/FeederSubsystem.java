package frc.robot.subsystems;

import java.util.HashMap;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {

    public enum FeedMode {
        STOPPED,
        INTAKE,
        PRESHOOT,
        SHOOT_ONE,
        CONTINUOUS,
        REVERSE_CONTINUOUS
    }

    private final double FEEDER_GEAR_RATIO_MULTIPLIER = 1;

    // Feeder PID constants
    
    private final double KP = 0.00008;
    private final double KI = 0.000001;
    private final double KD = 0.0009;

    private final int FEED_RPM_STOPPED = 0;
    private final int FEED_RPM_SHOOT = 2000; // how fast the feeder should be running when we are shooting
    private final int FEED_RPM_PRESHOOT = 1000; // how fast the feeder should be running when we are prepping shoot
    private final int FEED_RPM_INTAKE = 2000; // how fast the feeder should be running when indexing the balls
    private final int FEED_RPM_REVERSE_CONTINUOUS = -2500;

    // The number of revolutions of the feed motor required to cycle a ball all the
    // way from the feeders entry to the exit.
    public final int REV_PER_FULL_FEED = 1500;
    public final int EXIT_ADVANCE_REV = 0;

    // Subsystems internal data
    private CANSparkMax feedMotor;
    private RelativeEncoder feedEncoder;
    private SparkMaxPIDController feedPID;

    boolean exitSensorTripped = false;
    double advanceTargetPos = 0;

    private DigitalInput entrySensor;
    private DigitalInput exitSensor;

    private FeedModeBase currentMode;
    private HashMap<FeedMode, FeedModeBase> modes = new HashMap<FeedMode, FeedModeBase>();

    // network table entries for Shuffleboard
    private NetworkTableEntry feederRPMEntry;
    private NetworkTableEntry entrySensorEntry;
    private NetworkTableEntry exitSensorEntry;

    public FeederSubsystem(int feedMotorCanId, int entrySensorDio, int exitSensorDio) {

        feedMotor = new CANSparkMax(feedMotorCanId, MotorType.kBrushless);
        feedMotor.setIdleMode(IdleMode.kBrake);

        feedMotor.setSmartCurrentLimit(40);

        feedPID = feedMotor.getPIDController();
        feedPID.setP(KP);
        feedPID.setI(KI);
        feedPID.setD(KD);

        feedEncoder = feedMotor.getEncoder();
        feedEncoder.setPosition(0.0);
        feedEncoder.setVelocityConversionFactor(FEEDER_GEAR_RATIO_MULTIPLIER); // set feeder gear ratio

        // Sensors for Feeder
        entrySensor = new DigitalInput(entrySensorDio);
        exitSensor = new DigitalInput(exitSensorDio);

        // Setup our feed modes and initialize the system into the stopped mode.
        modes.put(FeedMode.STOPPED, new StoppedMode());
        modes.put(FeedMode.INTAKE, new IntakeMode());
        modes.put(FeedMode.PRESHOOT, new PreshootMode());
        modes.put(FeedMode.SHOOT_ONE, new ShootOneMode());
        modes.put(FeedMode.CONTINUOUS, new ContinuousMode());
        modes.put(FeedMode.REVERSE_CONTINUOUS, new ReverseContinuousMode());

        currentMode = modes.get(FeedMode.STOPPED);

        initTelemetry();
    }

    private void initTelemetry() {
        ShuffleboardTab tab = Shuffleboard.getTab("Shooter"); // Data is grouped with shooter and intake.

        feederRPMEntry = tab.add("Feeder RPM", 0)
                .withPosition(0, 1)
                .withSize(1, 1)
                .getEntry();

        entrySensorEntry = tab.add("Entry Sensor", false)
                .withPosition(1, 1)
                .withSize(1, 1)
                .getEntry();

        exitSensorEntry = tab.add("Exit Sensor", false)
                .withPosition(2, 1)
                .withSize(1, 1)
                .getEntry();
    }

    /**
     * Perform background processing for feeder system.
     */
    @Override
    public void periodic() {

        // We need to clear our exit sensor tripped flag once we no longer see a cargo
        // breaking the beam.
        if (exitSensor.get()) {
            exitSensorTripped = false;
        }

        // Execute the current mode, if it completes then put system in Stopped mode.
        if (currentMode.run(this)) {
            setFeedMode(FeedMode.STOPPED);
        }

        updateTelemetry();
    }

    private void updateTelemetry() {
        feederRPMEntry.setNumber(feedMotor.getEncoder().getVelocity());
        entrySensorEntry.forceSetBoolean(entrySensor.get());
        exitSensorEntry.forceSetBoolean(exitSensor.get());
    }

    /**
     * Tell caller if the feeder subsystem is currently idle (i.e. in stopped mode)
     */
    public boolean isIdle() {
        return currentMode.id == FeedMode.STOPPED;
    }

    /**
     * Set the current mode of the feeder system.
     * 
     * @param mode New mode for feeder.
     */
    public void setFeedMode(FeedMode mode) {

        currentMode.end(this); // Terminate the current mode
        currentMode = modes.get(mode); // Get the new desired mode
        currentMode.init(this); // Start the new mode
    }

    /**
     * Tell caller if there is a ball at the entry end of the feeder subsystem.
     * 
     * @return True if a ball is present at the entry sensor, false otherwise.
     */
    private boolean ballInEntry() {

        return !entrySensor.get();
    }

    /**
     * Tell caller if there is a at the exit end of the feeder. This is true
     * if the exit sensor has been tripped AND the
     * 
     * @return True if a ball is present at the exit sensor, false otherwise.
     */
    private boolean ballInExit() {
        boolean result = false;
        double currentPos = feedEncoder.getPosition();

        if (exitSensorTripped) {
            if (currentPos >= advanceTargetPos) {
                result = true;
            }
        } else {
            if (!exitSensor.get()) {
                exitSensorTripped = true;
                advanceTargetPos = currentPos + EXIT_ADVANCE_REV;
            }
        }

        return result;
    }

    /************************************************************************************************
     * Base class for all of our feed modes
     */
    private abstract class FeedModeBase {
        FeedMode id;

        private FeedModeBase(FeedMode id) {
            this.id = id;
        }

        protected void init(FeederSubsystem feeder) {
        }

        protected boolean run(FeederSubsystem feeder) {
            return false;
        }

        protected void end(FeederSubsystem feeder) {
        }
    }

    /************************************************************************************************
     * Implements our Stopped mode, stops motor in init and then runs forever.
     */
    private class StoppedMode extends FeedModeBase {
        private StoppedMode() {
            super(FeedMode.STOPPED);
        }

        @Override
        protected void init(FeederSubsystem feeder) {
            feeder.feedPID.setReference(FEED_RPM_STOPPED, ControlType.kVelocity);
        }
    }

    /************************************************************************************************
     * Implements Intake mode, advances feed wheels as long as there is a ball in
     * the entry and terminates once a ball reaches the exit end of the feeder.
     */
    private class IntakeMode extends FeedModeBase {
        private IntakeMode() {
            super(FeedMode.INTAKE);
        }

        @Override
        protected boolean run(FeederSubsystem feeder) {

            // If there is ball at the top of the feeder then stop the motor and exit
            // complete this mode.
            if (feeder.ballInExit()) {
                feeder.feedPID.setReference(FEED_RPM_STOPPED, ControlType.kVelocity);
                return true;
            }

            // If there is a ball in the intake end of the feeder then start the motor
            // otherwise stop it.
            if (feeder.ballInEntry()) {
                feeder.feedPID.setReference(FEED_RPM_INTAKE, ControlType.kVelocity);
            } else {
                feeder.feedPID.setReference(FEED_RPM_STOPPED, ControlType.kVelocity);
            }

            return false;
        }

        @Override
        protected void end(FeederSubsystem feeder) {
            feeder.feedPID.setReference(FEED_RPM_STOPPED, ControlType.kVelocity);
        }
    }

    /************************************************************************************************
     * Implements pre-Shoot mode, advances feeder until a ball is present at the
     * exit end of the feeder.
     */
    private class PreshootMode extends FeedModeBase {

        double targetPosition;

        private PreshootMode() {
            super(FeedMode.PRESHOOT);
        }

        @Override
        protected void init(FeederSubsystem feeder) {
            if(!feeder.ballInExit()) {
                targetPosition = feedEncoder.getPosition() + REV_PER_FULL_FEED;
                feeder.feedPID.setReference(FEED_RPM_PRESHOOT, ControlType.kVelocity);
            }
        }

        @Override
        protected boolean run(FeederSubsystem feeder) {

            // If we see a ball at the exit sensor then we have moved them to the top of the
            // feeder and ready to shoot.
            if (feeder.ballInExit()) {
                return true;
            }

            // If the exit sensor has not seen a ball yet but the belt has moved the full
            // length of the feeder then there are no balls, bail out.
            if (feeder.feedEncoder.getPosition() >= targetPosition) {
                return true;
            }

            return false;
        }

        @Override
        protected void end(FeederSubsystem feeder) {
            feeder.feedPID.setReference(FEED_RPM_STOPPED, ControlType.kVelocity);
        }
    }

    /************************************************************************************************
     * Implements Shoot mode, advances one ball out of the feeder into the shooter,
     * advances next ball (if there is one) to the top of the feeder then
     * terminates.
     */
    private class ShootOneMode extends FeedModeBase {
        private boolean gapSeen;

        private ShootOneMode() {
            super(FeedMode.SHOOT_ONE);
        }

        @Override
        protected void init(FeederSubsystem feeder) {

            gapSeen = false;
            feeder.feedPID.setReference(FEED_RPM_SHOOT, ControlType.kVelocity);
        }

        @Override
        protected boolean run(FeederSubsystem feeder) {

            boolean result = false;

            if (gapSeen) {
                if (feeder.ballInExit()) {
                    result = true;
                }
            } else {
                if (!feeder.ballInExit()) {
                    gapSeen = true;
                }
            }

            return result;
        }

        @Override
        protected void end(FeederSubsystem feeder) {
            feeder.feedPID.setReference(FEED_RPM_STOPPED, ControlType.kVelocity);
        }
    }

    /************************************************************************************************
     * Implements Continuous mode, advanced the feeder forever.
     */
    private class ContinuousMode extends FeedModeBase {

        private ContinuousMode() {
            super(FeedMode.CONTINUOUS);
        }

        @Override
        protected void init(FeederSubsystem feeder) {
            feeder.feedPID.setReference(FEED_RPM_SHOOT, ControlType.kVelocity);
        }

        @Override
        protected void end(FeederSubsystem feeder) {
            feeder.feedPID.setReference(FEED_RPM_STOPPED, ControlType.kVelocity);
        }
    }

    /************************************************************************************************
     * Implements ReverseContinuous mode, reverse the feeder forever.
     */
    private class ReverseContinuousMode extends FeedModeBase {

        private ReverseContinuousMode() {
            super(FeedMode.REVERSE_CONTINUOUS);
        }

        @Override
        protected void init(FeederSubsystem feeder) {
            feeder.feedPID.setReference(FEED_RPM_REVERSE_CONTINUOUS, ControlType.kVelocity);
        }

        @Override
        protected void end(FeederSubsystem feeder) {
            feeder.feedPID.setReference(FEED_RPM_STOPPED, ControlType.kVelocity);
        }
    }
}
