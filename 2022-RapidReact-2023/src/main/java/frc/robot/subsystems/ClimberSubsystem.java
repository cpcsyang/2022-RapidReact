// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  public enum HookPosition {
    Grab,
    Release,
    Lock
  };

  public enum WindmillState {
    Home,
    Freeze,
    FirstToSecond,
    SecondToThird,
    ShiftWeightOffFirst,
    ShiftWeightOffSecond,
    Hang
  };

  public enum HookSet {
    Red,
    Blue
  }

  public HookPosition currentHookPosition;
  public WindmillState currentWindmillState;
  public HookSet currentHookSet;

  // Hook Helper class.
  public class Hook {

    public RelativeEncoder hookPosition;
    private CANSparkMax hookMotor;
    private SparkMaxPIDController hookPositionPID;

    // PID values
    private double hookP = 0.1;
    private double hookI = 0;
    private double hookD = 0;
    private double hookIz = 0;
    private double hookFF = 0;

    // LimitSwitches
    public DigitalInput hookLimitSwitch1;
    public DigitalInput hookLimitSwitch2;

    // Hook Constants
    private final double ROTATIONS_PER_DEGREE = (270.0 / 1.0) * (1.0 / 360.0);
    private final double kHookMinOutput = -1;
    private final double kHookMaxOutput = 1;

    // positions based on encoders
    private double release = 195;
    private double grab = 20;
    private double lock = 0;

    // Tracking Info
    public boolean goingHome = true;
    private double targetAngle  = 0;

    // All of these args are in Degreas
    public Hook(int HookCanId, HookSet hookSet) {
      hookMotor = new CANSparkMax(HookCanId, MotorType.kBrushless);
      hookPosition = hookMotor.getEncoder();
      hookPositionPID = hookMotor.getPIDController();
      
      hookPosition.setPositionConversionFactor(ROTATIONS_PER_DEGREE);

      hookMotor.setInverted(true);

      // Setting the PID Values
      hookPositionPID.setP(hookP);
      hookPositionPID.setI(hookI);
      hookPositionPID.setD(hookD);
      hookPositionPID.setIZone(hookIz);
      hookPositionPID.setFF(hookFF);
      hookPositionPID.setOutputRange(kHookMinOutput, kHookMaxOutput);

      hookMotor.setIdleMode(IdleMode.kBrake);
    }

    public void zeroEncoder(){
      hookPosition.setPosition(0);
    }

    public void setAngle(double angle) {
      targetAngle = angle;
      hookPositionPID.setReference(targetAngle, ControlType.kPosition);
    }

    public void setSpeed(double speed){
      hookMotor.set(speed);
    }

    public double getAngle(){
      return hookPosition.getPosition();
    }

    public double getVelocity(){
      return hookPosition.getVelocity();
    }

    public void setTargetAngle(double angle){
      hookPositionPID.setReference(angle, ControlType.kPosition);
    }

    public void setHookPosition(HookPosition position) {
      switch (position) {
        case Grab:
          setAngle(grab);
          currentHookPosition = HookPosition.Grab;
          break;
        case Release:
          setAngle(release);
          currentHookPosition = HookPosition.Release;
          break;
        case Lock:
          setAngle(lock);
          currentHookPosition = HookPosition.Lock;
          break;
      }
    }
    
    public boolean isAtTargetPosition() {
      return  Math.abs(targetAngle - getAngle()) < 2;
    }
  }

  public class Windmill {
    // Phisical controllers
    private CANSparkMax windmillMotor;
    private CANSparkMax windmillFollowerMotor;
    private RelativeEncoder windmillEncoder;
    private SparkMaxPIDController windmillPIDController;

    // Phisical Offsets and speeds
    private double windmillRotationSpeed = 0.5; // :) 

    // Windmill Constants
    private final double ROTATIONS_PER_DEGREE = (366.66 / 1.0) * (1.0 / 360.0) * (360.0 / 500.0) * (180.0 / 132.0);

    // Windmill Positions
    private double targetAngle;

    private final double HOME = 0;
    private final double FIRST_TO_SECOND = 165; //155
    private final double SHIFT_WEIGHT_FIRST_OFFSET = -55; //-55
    private final double SHIFT_WEIGHT_SECOND_OFFSET = -45; // lower this one when able to test
    private final double SECOND_TO_THIRD = FIRST_TO_SECOND + 185; //fts + 180 // changed from 195
    private final double HANG_OFFSET = 30; // tune this

    // PID Values
    private double windmillSmartP = 0.0005; //0.1
    private double windmillI = 0.0;
    private double windmillD = 0.0;

    private double windmillPoseP = 0.1;

    private boolean homed = false;

    public Windmill(int WindmillCanId, int WindmillFollowerCanId, int WindmillLimitSwitchId) {
      // Creating Objects
      windmillMotor = new CANSparkMax(WindmillCanId, MotorType.kBrushless);
      windmillFollowerMotor = new CANSparkMax(WindmillFollowerCanId, MotorType.kBrushless);

      // Setting Modes
      windmillFollowerMotor.follow(windmillMotor, true);
      // windmillFollowerMotor.setIdleMode(IdleMode.kBrake);
      // windmillMotor.setIdleMode(IdleMode.kBrake);
      windmillFollowerMotor.setIdleMode(IdleMode.kBrake);
      windmillMotor.setIdleMode(IdleMode.kBrake);

      // Setting Local Varbles
      windmillPIDController = windmillMotor.getPIDController();
      windmillEncoder = windmillMotor.getEncoder();

      windmillEncoder.setPositionConversionFactor(ROTATIONS_PER_DEGREE);

      windmillPIDController.setSmartMotionMaxVelocity(4200, 0); // 35
      windmillPIDController.setSmartMotionMaxAccel(2500, 0); // 15


      // Setting PIDs
      windmillPIDController.setP(windmillSmartP);
      windmillPIDController.setI(windmillI);
      windmillPIDController.setD(windmillD);
      windmillPIDController.setOutputRange(-windmillRotationSpeed, windmillRotationSpeed);

      windmillEncoder.setPosition(0);
    }

    public void setAngle(double angle) {
      windmillPIDController.setP(windmillSmartP);

      targetAngle = angle;
      windmillPIDController.setReference(targetAngle, ControlType.kSmartMotion); //controltype smart motion
    }

    
    public void setPose(double pose) {
      windmillPIDController.setP(windmillPoseP);

      targetAngle = pose;
      windmillPIDController.setReference(targetAngle, ControlType.kPosition); 
    }
    

    public double getAngle(){
      return windmillEncoder.getPosition();
    }

    public double getTargetAngle(){
      return targetAngle;
    }

    public void setWindmillOutput(double speed){
      windmillMotor.set(speed);
    }

    public void setHomeStatus(boolean state){
      homed = state;
    }

    public boolean getHomeSatus(){
      return homed;
    }

    public void rotateWindmill(WindmillState position) {
      if(homed){
        switch (position) {
          case Freeze:
            setAngle(targetAngle);
            currentWindmillState = WindmillState.Home;
            break;
          case Home:
            setAngle(HOME);
            currentWindmillState = WindmillState.Home;
            break;
          case FirstToSecond:
            setAngle(FIRST_TO_SECOND);
            currentWindmillState = WindmillState.FirstToSecond;
            break;
          case ShiftWeightOffFirst:
            // setAngle(FIRST_TO_SECOND + SHIFT_WEIGHT_FIRST_OFFSET);
            setPose(FIRST_TO_SECOND + SHIFT_WEIGHT_FIRST_OFFSET);
            currentWindmillState = WindmillState.ShiftWeightOffFirst;
            break;
          case SecondToThird:
            setAngle(SECOND_TO_THIRD);
            currentWindmillState = WindmillState.SecondToThird;
            break;
          case ShiftWeightOffSecond:
            // setAngle(SECOND_TO_THIRD + SHIFT_WEIGHT_SECOND_OFFSET);
            setPose(SECOND_TO_THIRD + SHIFT_WEIGHT_SECOND_OFFSET);
            currentWindmillState = WindmillState.ShiftWeightOffSecond;
            break;
          case Hang:
            // setAngle(SECOND_TO_THIRD + HANG_OFFSET);
            setPose(SECOND_TO_THIRD + HANG_OFFSET);
            currentWindmillState = WindmillState.Hang;
            break;
        }
      }
    }

    public boolean isAtTargetPosition() {
      return  Math.abs(targetAngle - getAngle()) < 3;
    }
  }

  public class Elevator {

    private CANSparkMax elevatorMotor;
    private RelativeEncoder elevatorEncoder;
    private SparkMaxPIDController elevatorPIDController;

    // Elevator PID
    private double elevatorP = 0.01;
    private double elevatorI;
    private double elevatorD;

    // Gear Ratio
    private double gearRatio = 1;

    public Elevator(int ElevatorCanId) {
      elevatorMotor = new CANSparkMax(ElevatorCanId, MotorType.kBrushless);

      elevatorEncoder = elevatorMotor.getEncoder();
      elevatorPIDController = elevatorMotor.getPIDController();

      elevatorEncoder.setPositionConversionFactor(gearRatio);
      elevatorMotor.setInverted(false);

      elevatorMotor.setIdleMode(IdleMode.kBrake);

      elevatorPIDController.setP(elevatorP);
      elevatorPIDController.setI(elevatorI);
      elevatorPIDController.setD(elevatorD);
    }

    public void zeroEncoder(){
      elevatorEncoder.setPosition(0);
    }

    public double getVelocity(){
      return elevatorEncoder.getVelocity();
    }

    public void extendElevator(double speed){
      elevatorMotor.set(speed);
    }

    public double getHeight(){
      return elevatorEncoder.getPosition();
    }

    public void setTargetHeight(double height){
      elevatorPIDController.setReference(height, ControlType.kPosition);
    }
  }

  public Hook hookRed;
  public Hook hookBlue;
  public Windmill windmill;
  public Elevator elevator;

  // Shuffleboard Entrys
  private NetworkTableEntry redHookCurrentAngleEntry;
  private NetworkTableEntry redHookTargetAngleEntry;
  private NetworkTableEntry blueHookCurrentAngleEntry;
  private NetworkTableEntry blueHookTargetAngleEntry;
  private NetworkTableEntry redHookHomed;
  private NetworkTableEntry blueHookHomed;

  private NetworkTableEntry windmillCurrentAngleEntry;
  private NetworkTableEntry windmillTargetAngleEntry;

  private NetworkTableEntry elevatorCurrentAngleEntry;


  public ClimberSubsystem(int ElevatorCanId, int WindmillCanId, int WindmillFollowerCanId, int HookABCanId, int HookXYCanId,
      int WindmillLimitSwitchId) {

    hookRed = new Hook(HookABCanId, HookSet.Red);
    hookBlue = new Hook(HookXYCanId, HookSet.Blue);
    windmill = new Windmill(WindmillCanId, WindmillFollowerCanId, WindmillLimitSwitchId);
    elevator = new Elevator(ElevatorCanId);

    initTelemetry();
  }

  @Override
  public void periodic() {
    updateTelemetry();
  }

  private void initTelemetry() {
    ShuffleboardTab tab = Shuffleboard.getTab("Climber"); 

    // ELEVATOR
    elevatorCurrentAngleEntry = tab.add("Elevator Position", 0)
            .withPosition(1, 5)
            .withSize(1, 1)
            .getEntry();

    // WINDMILL

    // HOOKS
    redHookHomed = tab.add("Red Hook Zeroed", false)
            .withPosition(1, 2)
            .withSize(1, 1)
            .getEntry();

    blueHookHomed = tab.add("Blue Hook Zeroed", false)
            .withPosition(2, 2)
            .withSize(1, 1)
            .getEntry();
    
    redHookCurrentAngleEntry = tab.add("Red Hook Current Angle", 0)
            .withPosition(5, 2)
            .withSize(1, 1)
            .getEntry();

    redHookTargetAngleEntry = tab.add("Red Hook Target Angle", 0)
            .withPosition(6, 2)
            .withSize(1, 1)
            .getEntry();
    
    blueHookCurrentAngleEntry = tab.add("Blue Hook Current Angle", 0)
            .withPosition(7, 2)
            .withSize(1, 1)
            .getEntry();

    blueHookTargetAngleEntry = tab.add("Blue Hook Target Angle", 0)
            .withPosition(8, 2)
            .withSize(1, 1)
            .getEntry();

    windmillTargetAngleEntry = tab.add("Windmill Target Angle", 0)
            .withPosition(5, 1)
            .withSize(1, 1)
            .getEntry();

    windmillCurrentAngleEntry = tab.add("Windmill current Angle", 0)
            .withPosition(6, 1)
            .withSize(1, 1)
            .getEntry();
  }

  private void updateTelemetry() {
    redHookCurrentAngleEntry.setDouble(hookRed.getAngle());
    blueHookCurrentAngleEntry.setDouble(hookBlue.getAngle());
    redHookTargetAngleEntry.setDouble(hookRed.targetAngle);
    blueHookTargetAngleEntry.setDouble(hookBlue.targetAngle);
    redHookHomed.setBoolean(!hookRed.goingHome);
    blueHookHomed.setBoolean(!hookBlue.goingHome);

    windmillCurrentAngleEntry.setDouble(windmill.getAngle());
    windmillTargetAngleEntry.setDouble(windmill.targetAngle);

    elevatorCurrentAngleEntry.setDouble(elevator.getHeight());
}
}

//smart motion