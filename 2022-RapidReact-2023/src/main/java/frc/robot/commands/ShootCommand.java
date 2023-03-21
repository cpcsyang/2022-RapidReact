package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeedMode;

public class ShootCommand extends CommandBase {

    private ShooterSubsystem shooter;
    private FeederSubsystem feeder;
    private LimelightSubsystem limelight;

    private Consumer<Boolean> shootReadyNotifier;
    private boolean manualShoot;

    private BooleanSupplier shootTrigger;
    private BooleanSupplier forceShootTrigger;

    private String shootingPose;
    private boolean varyingRange;

    private boolean stagingCargo;

    private boolean atSpeed;
    private boolean aligned;
    private boolean trigger;

    // fixed range & manual shoot
    public ShootCommand(ShooterSubsystem shooter, FeederSubsystem feeder, 
                        Consumer<Boolean> shootReadyNotifier, BooleanSupplier shootTrigger,
                        BooleanSupplier forceShootTrigger,
                        String shootingPose) {
        this.shooter = shooter;
        this.feeder = feeder;

        this.shootReadyNotifier = shootReadyNotifier;
        this.manualShoot = true;

        this.shootTrigger = shootTrigger;
        this.forceShootTrigger = forceShootTrigger;

        this.shootingPose = shootingPose;
        this.varyingRange = false;

        addRequirements(shooter, feeder);
    }

    // varying range & manual shoot
    public ShootCommand(ShooterSubsystem shooter, FeederSubsystem feeder, LimelightSubsystem limelight,
                        Consumer<Boolean> shootReadyNotifier, BooleanSupplier shootTrigger,
                        BooleanSupplier forceShootTrigger) {
        this(shooter, feeder, shootReadyNotifier, shootTrigger, forceShootTrigger, "");

        this.limelight = limelight;
        this.varyingRange = true;
    }

    // varying range & auto shoot
    public ShootCommand(ShooterSubsystem shooter, FeederSubsystem feeder, LimelightSubsystem limelight,
                        BooleanSupplier forceShootTrigger) {
        this(shooter, feeder, null, null, forceShootTrigger, "");

        this.limelight = limelight;
        this.varyingRange = true;
        this.manualShoot = false;
    }

    @Override
    public void initialize() {
        // send the cargo up to feeder's exit
        feeder.setFeedMode(FeedMode.PRESHOOT);
        stagingCargo = true;

        // initialize rumble
        if (manualShoot) {
            shootReadyNotifier.accept(false);
        }

        // turn on limelight
        if (varyingRange) {
            limelight.setLEDMode(limelight.LED_ON);
        }
        // set fixed range
        else {
            shooter.setRange(shootingPose);
        }
    }

    @Override
    public void execute() {

        // update range with limelight
        if (varyingRange) {
            shooter.setRange(limelight.getDistance());
        }

        // check if cargo has arrived at feeder's exit
        if (stagingCargo) {
            if (feeder.isIdle()) {
                stagingCargo = false;
            }
            else {
                return;
            }
        }

        // check if shooter is ready
        if (varyingRange) {
            aligned = limelight.aligned();
        }
        else {
            aligned = true; // do not check for alignment if fixed range
        }

        atSpeed = shooter.ready();

        if (manualShoot) {
            trigger = shootTrigger.getAsBoolean();
        }
        else {
            trigger = true; // auto shoot does not wait for trigger
        }
        

        // rumble if at speed
        if (manualShoot) {
            shootReadyNotifier.accept(atSpeed);
        }

        // shuffleboard aligned
        shooter.aligned = aligned;

        // shoot
        if (trigger && atSpeed && aligned) { 
            feeder.setFeedMode(FeedMode.CONTINUOUS);
        }
        else if (forceShootTrigger.getAsBoolean()) {
            feeder.setFeedMode(FeedMode.CONTINUOUS);
        }
        else {
            feeder.setFeedMode(FeedMode.STOPPED);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // stop feeder
        feeder.setFeedMode(FeedMode.STOPPED);

        // disable rumble
        if (manualShoot) {
            shootReadyNotifier.accept(false);
        }

        // turn off limelight
        if (varyingRange) {
            limelight.setLEDMode(limelight.LED_OFF);
        }
    }
}
