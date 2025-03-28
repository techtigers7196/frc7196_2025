package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

//Libraries for motor controllers
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig; 


//Libraries for Elevator
import frc.robot.Constants.ElevatorSubsystemConstants;
import frc.robot.Constants.ElevatorSetpoints;
import frc.robot.Constants.ShootConstants;

//Smart Dashboard library 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Libraries for Coral Intake Limit Switch 
import edu.wpi.first.wpilibj.DigitalInput;

//Libraries for Time of Flight Sensor
//https://www.playingwithfusion.com/frc/playingwithfusion2025.json
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import frc.robot.Constants.winch;


public class ElevatorSubsystem extends SubsystemBase{

  public enum Setpoint {
      kLevel1Intake,
      kLevel2,
      kLevel3,
      kLevel4;
  }

  //motor controllers 
  private SparkMax elevatorMotor = new SparkMax (ElevatorSubsystemConstants.kelevatorMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController elevatorClosedLoopController = elevatorMotor.getClosedLoopController();
  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
  private final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
  public static final SparkMaxConfig shootConfig = new SparkMaxConfig();

  private SparkMax winchMotor = new SparkMax (winch.kwinchMotorCanId, MotorType.kBrushed);

//PID variables for manual PID
private final double kP = 0.2;
private final double kI = 0;
private final double kD = 0.08;
private final double kfeedforward = 0.003;

//PID controller
private final PIDController pid = new PIDController(kP, kI, kD);

  //shooting controllers 
  private SparkMax shootMotor = new SparkMax(ShootConstants.kshootMotorCanId,MotorType.kBrushless);
  
  private DigitalInput coralIntakeLimitSwitch = new DigitalInput(ShootConstants.kCoralIntakeLimitSwitchPort);
  private TimeOfFlight coralTimeOfFlight;



  // Member variables for subsystem state management
  //   private boolean wasResetByButton = false;
  //   private boolean wasResetByLimit = false;
  private double elevatorCurrentTarget = ElevatorSetpoints.kLevel1Intake;
  ElevatorFeedforward feedforward = new ElevatorFeedforward(ElevatorSubsystemConstants.kS, 
    ElevatorSubsystemConstants.kG,
    ElevatorSubsystemConstants.kV,
    ElevatorSubsystemConstants.kA
  );

  private TrapezoidProfile.Constraints constraints;
  private TrapezoidProfile.State goalState;
  private TrapezoidProfile.State nextState;
  private TrapezoidProfile profile;

  //sets the PID VALUES
  public ElevatorSubsystem(){
    // Configure basic settings of the elevator motor
    elevatorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12);
    shootConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(30).voltageCompensation(12);


    coralTimeOfFlight = new TimeOfFlight(ShootConstants.kcoralTimeOfFlightPort);
    coralTimeOfFlight.setRangingMode(RangingMode.Short, 24);
  

    /*
    * Configure the closed loop controller. We want to make sure we set the
    * feedback sensor as the primary encoder.
    */
    elevatorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control
        .p(ElevatorSubsystemConstants.kP)
        .d(ElevatorSubsystemConstants.kD)
        .velocityFF(ElevatorSubsystemConstants.velocityFF)
        .outputRange(ElevatorSubsystemConstants.outputRangeMin,ElevatorSubsystemConstants.outputRangeMax)
        .maxMotion
        // Set MAXMotion parameters for position control
        .maxVelocity(ElevatorSubsystemConstants.maxVelocity)
        .maxAcceleration(ElevatorSubsystemConstants.maxAcceleration)
        .allowedClosedLoopError(ElevatorSubsystemConstants.allowedClosedLoopError);

    elevatorMotor.configure(
      elevatorConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    // Zero elevator encoders on initialization
    elevatorEncoder.setPosition(0);

    shootMotor.configure(
      shootConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    constraints = new TrapezoidProfile.Constraints(
      ElevatorSubsystemConstants.maxVelocity,
      ElevatorSubsystemConstants.maxAcceleration);
    nextState = new TrapezoidProfile.State(0,0);
    goalState = new TrapezoidProfile.State(0,0);
    profile = new TrapezoidProfile(constraints);

  }

  /** Set the intake motor power in the range of [-1, 1]. */
  private void setShootPower(double power) {
    shootMotor.set(power);
  }

  public void moveToSetpointMaxMotion() {
    elevatorClosedLoopController.setReference(elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

  public void moveToSetpointPID(){
    double pidValue = pid.calculate(elevatorEncoder.getPosition(),elevatorCurrentTarget);
    double power = kfeedforward+pidValue;
    elevatorMotor.set(power);
  }

  public double getDistance(){
    return coralTimeOfFlight.getRange();
  }

  public boolean hasCoralInElevatorPath() {
    if(this.getDistance() < ElevatorSubsystemConstants.intakeWidth && elevatorEncoder.getPosition() < 0.5){
      return true;
    }

    return false;
  }

  public boolean isCoralLoaded() {
    // return !this.coralIntakeLimitSwitch.get() && !this.hasCoralInElevatorPath();
    return !this.coralIntakeLimitSwitch.get() && !this.hasCoralInElevatorPath();
  }

  public Command setSetpointCommand(Setpoint setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            case kLevel1Intake: 
              elevatorCurrentTarget = ElevatorSetpoints.kLevel1Intake;
              break;
            case kLevel2:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel2;
              break;
            case kLevel3:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel3;
              break;
            case kLevel4:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel4;
              break;
          }
          
          goalState = new TrapezoidProfile.State(elevatorCurrentTarget,0);
        }).unless(() -> this.hasCoralInElevatorPath());
  }

  //command to shoot the coral if there is a coral loaded, if not, motor will not run.
  public Command runShootCommandWithSwitch(){
    return Commands.parallel(this.run(
      ()-> this.setShootPower(ShootConstants.shootPower)
    ).until(
      () -> !this.isCoralLoaded()
    ), new WaitCommand(0.5))
    .andThen(
      ()-> this.setShootPower(0)
    );
  }

  // ParallelCommandGroup
  // raceCommandGroup name = new raceCommandGroup(new waitCOmmand(1), other command)

  //Sets the motor on, once the Coral hits the swith the motor will stop the intake
  public Command intakeCommandWithSwitch(){
    return this.run(
      ()-> this.setShootPower(ShootConstants.intakePower)
    ).until(
      () -> this.isCoralLoaded()
    ).andThen(
      ()-> this.setShootPower(0)
    );
  }

   /**
   * Command to run the intake motor. When the command is interrupted, e.g. the button is released,
   * the motor will stop.
   */
  public Command runShootCommand() {
    return this.startEnd(
        () -> this.setShootPower(ShootConstants.shootPower), () -> this.setShootPower(0.0));
  }

  /**
   * Command to reverses the intake motor. When the command is interrupted, e.g. the button is
   * released, the motor will stop.
   */
  public Command reverseIntakeCommand() {
    return this.startEnd(
        () -> this.setShootPower(-ShootConstants.intakePower), () -> this.setShootPower(0.0));
  }

  //Command to itake the coral regardless of the limit switch
  public Command intakeCommand() {
    return this.startEnd(
        () -> this.setShootPower(ShootConstants.intakePower), () -> this.setShootPower(0.0));
  }

  /**
   * Command to run the intake motor. When the command is interrupted, e.g. the button is released,
   * the motor will stop.
   */
  public Command L1shootCommand() {
    return this.startEnd(
        () -> this.setShootPower(ShootConstants.L1shootPower), () -> this.setShootPower(0.0));
  }

  public Command retractIntake() {
    return this.startEnd(
    () -> winchMotor.set(-0.2), () -> winchMotor.set(0.0));
}

public Command unretractIntake() {
    return this.startEnd(
    () -> winchMotor.set(0.1), () -> winchMotor.set(0.0));
}

  @Override
  public void periodic(){
    SmartDashboard.putNumber("Coral/Elevator/Target Position", elevatorCurrentTarget);
    SmartDashboard.putNumber("Coral/Elevator/Actual Position", elevatorEncoder.getPosition());
    SmartDashboard.putNumber("Coral/Shooter/TOFDist", this.getDistance());
    SmartDashboard.putBoolean("Coral/Shooter/BlockingElevator", this.hasCoralInElevatorPath());
    SmartDashboard.putBoolean("Coral/Shooter/IsLoaded", this.isCoralLoaded());
    moveToSetpointMaxMotion();
  }
}

