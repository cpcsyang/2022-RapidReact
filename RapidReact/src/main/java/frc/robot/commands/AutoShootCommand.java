package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeedMode;

public class AutoShootCommand extends CommandBase {

    private ShooterSubsystem shooter;
    private FeederSubsystem feeder;
    private LimelightSubsystem limelight = null;
    private double currentRange;
    private boolean stagingCargo;
    private Timer timer = new Timer();
    private double autoTimer;
    private boolean atSpeed;

    // Fixed range version, take the range to target as a parameter
    public AutoShootCommand(ShooterSubsystem shooter, FeederSubsystem feeder,
                            double range,
                            double autoTimer) {
        this.shooter = shooter;
        this.feeder = feeder;
        this.currentRange = range;
        this.autoTimer = autoTimer;

        addRequirements(shooter, feeder);
    }

    // Variable range version, takes a limelight object that is used to determine
    // the range
    public AutoShootCommand(ShooterSubsystem shooter, FeederSubsystem feeder,
                            LimelightSubsystem limelight,
                            double autoTimer) {
        this(shooter, feeder, 0, autoTimer);

        this.limelight = limelight;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        feeder.setFeedMode(FeedMode.PRESHOOT);
        stagingCargo = true;

        if (limelight != null) {
            limelight.setLEDMode(limelight.LED_ON);
        }

        // Initialze the shooter range, if we have a limelight it will get updated each
        // time through periodic.
        shooter.setRange(currentRange);
        
        timer.reset();
        timer.start();

        atSpeed = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // If we have a limelight then use it to update the current range to target
        if (limelight != null) {
            currentRange = limelight.getDistance();
            shooter.setRange(currentRange);
        }

        // We bail out here if we are staging cargo and the feeder has not stopped yet.
        if (stagingCargo) {
            if (feeder.isIdle()) {
                stagingCargo = false;
            } else {
                return;
            }
        }

        boolean aligned = true;
        if (limelight != null) {
            aligned = limelight.aligned();
        }
        boolean atSpeed = shooter.ready();

        // shuffleboard aligned
        shooter.aligned = aligned;

        // We only get here if cargo staging has completed.
        if (atSpeed) {
            feeder.setFeedMode(FeedMode.CONTINUOUS);
        }
        else {
            feeder.setFeedMode(FeedMode.STOPPED);
        }
      
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        feeder.setFeedMode(FeedMode.STOPPED);

        timer.stop();

        if (limelight != null) {
            limelight.setLEDMode(limelight.LED_OFF);
        }

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(autoTimer);
    }
}
