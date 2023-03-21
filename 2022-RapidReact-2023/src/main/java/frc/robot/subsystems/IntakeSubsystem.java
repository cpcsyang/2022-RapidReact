package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

public class IntakeSubsystem extends SubsystemBase {

  public enum IntakeState {
    RETRACTED,
    EXTENDING,
    EXTENDED,
    RETRACTING
  }

  private IntakeState currentState = IntakeState.RETRACTED;
  private CANSparkMax intakeMotor;
  private final double POWER = 0.8;
  private final DoubleSolenoid boomIntakeSolenoid;
  private final DoubleSolenoid armIntakeSolenoid;
  private boolean boomIsOut;
  private boolean armIsOut;
  private final int MOTOR_CURRENT_LIMIT = 60;
  private double startTime = 0;
  private final double DELAYTIME = 0.5;

  // network table entries for Shuffleboard
  private NetworkTableEntry boomIsOutEntry;
  private NetworkTableEntry armIsOutEntry;
  private NetworkTableEntry intakeMotorSpeedEntry;

  private NetworkTableEntry currentSpeedEntry;


  /** Creates a new instance of the Shooter subsystem. */
  /**
   * Boom is the upper-arm of the intake, Arm is the fore-arm of the intake
   * 
   * @param motor1CANId
   * @param boomRetractSolenoidChan
   * @param boomExtendSolenoidChan
   * @param armRetractSolenoidChan
   * @param armExtendSolenoidChan
   */
  public IntakeSubsystem(int motor1CANId,
      int boomRetractSolenoidChan, int boomExtendSolenoidChan,
      int armRetractSolenoidChan, int armExtendSolenoidChan) {

    intakeMotor = new CANSparkMax(motor1CANId, MotorType.kBrushless);
    intakeMotor.setIdleMode(IdleMode.kCoast);
    intakeMotor.setSmartCurrentLimit(MOTOR_CURRENT_LIMIT);
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);

    boomIntakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, boomExtendSolenoidChan,
        boomRetractSolenoidChan);
    armIntakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, armExtendSolenoidChan, armRetractSolenoidChan);

    initTelemetry();
  }

  @Override
  public void periodic() {
    switch (currentState) {
      case EXTENDING:
        extendArm();
        if(!boomIsOut) extendBoom();
        if(boomIsOut && Timer.getFPGATimestamp() - startTime > 2 * DELAYTIME) {
          spinBallIn();
          currentState = IntakeState.EXTENDED;
        }
        break;
      case EXTENDED:
        break;
      case RETRACTING:
        stopMotor();
        if(boomIsOut)retractBoom();
        if(!boomIsOut && Timer.getFPGATimestamp() - startTime > DELAYTIME /2){
          retractArm();
          currentState = IntakeState.RETRACTED;
        }
        break;
      case RETRACTED:
        break;    
    }
    updateTelemetry();
  }

  public IntakeState getCurrentState(){
    return currentState;
  }

  // ---------------------------------------------------------------------------
  // Intake control methods
  // ---------------------------------------------------------------------------

  public void extend() {
    startTime = Timer.getFPGATimestamp();
    currentState = IntakeState.EXTENDING;
  }

  public void retract() {
    startTime = Timer.getFPGATimestamp();
    currentState = IntakeState.RETRACTING;
  }

  // ---------------------------------------------------------------------------
  // Solenoid movement methods
  // ---------------------------------------------------------------------------

  public void extendBoom() {
    boomIntakeSolenoid.set(Value.kForward);
    boomIsOut = true;
  }

  public void retractBoom() {
    boomIntakeSolenoid.set(Value.kReverse);
    boomIsOut = false;
  }

  public void extendArm() {
    armIntakeSolenoid.set(Value.kForward);
    armIsOut = true;
  }

  public void retractArm() {
    armIntakeSolenoid.set(Value.kReverse);
    armIsOut = false;
  }

  // ---------------------------------------------------------------------------
  // Intake motor control methods 
  // ---------------------------------------------------------------------------

  public void spinBallIn() {
    if(boomIsOut && armIsOut)
    intakeMotor.set(POWER);
  } 

  public void operatorBallIn() {
    intakeMotor.set(POWER);
  }


  public void spinBallOut() {
    if(boomIsOut && armIsOut)
      intakeMotor.set(-POWER);
  }

  public void operatorBallOut() {
    intakeMotor.set(-POWER);
  }

  public void stopMotor() {
    intakeMotor.set(0);
  }

  // ---------------------------------------------------------------------------
  // Telemetry
  // ---------------------------------------------------------------------------

  private void initTelemetry() {
    ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

    intakeMotorSpeedEntry = tab.add("Motor Speed", 0)
        .withPosition(0, 0)
        .withSize(1, 1)
        .getEntry();

    boomIsOutEntry = tab.add("Boom Extended", false)
        .withPosition(1, 0)
        .withSize(1, 1)
        .getEntry();

    armIsOutEntry = tab.add("Arm Extended", false)
        .withPosition(2, 0)
        .withSize(1, 1)
        .getEntry();


    currentSpeedEntry = tab.add("current speed", false)
        .withPosition(3, 0)
        .withSize(1, 1)
        .getEntry();
  }

  private void updateTelemetry() {
    intakeMotorSpeedEntry.setValue(intakeMotor.get());
    boomIsOutEntry.setBoolean(boomIsOut);
    armIsOutEntry.setBoolean(armIsOut);
    currentSpeedEntry.setValue(intakeMotor.getOutputCurrent());
  }
}
