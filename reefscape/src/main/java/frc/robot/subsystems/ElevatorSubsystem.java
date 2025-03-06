package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.math.controller.ElevatorFeedforward;

//Libraries for Time of Flight Sensor
// https://www.playingwithfusion.com/frc/playingwithfusion2025.json
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

//Smart Dashboard library 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Libraries for Coral Intake Limit Switch
import edu.wpi.first.wpilibj.DigitalInput;

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

  //shooting controllers 
  private SparkMax shootMotor = new SparkMax(ShootConstants.kshootMotorCanId,MotorType.kBrushless);

  private DigitalInput coralIntakeLimitSwitch = new DigitalInput(ShootConstants.kCoralIntakeLimitSwitchPort); 
  private TimeOfFlight coralTimeOfFlight;

  private double elevatorCurrentTarget = ElevatorSetpoints.kLevel1Intake;
  private ElevatorFeedforward feedforward = new ElevatorFeedforward(ElevatorSubsystemConstants.kS, 
                                                                    ElevatorSubsystemConstants.kG,
                                                                    ElevatorSubsystemConstants.kV,
                                                                    ElevatorSubsystemConstants.kA);

  private TrapezoidProfile.Constraints constraints;
  private TrapezoidProfile.State goalState;
  private TrapezoidProfile.State nextState;
  private TrapezoidProfile profile;

  public ElevatorSubsystem(){
    // Configure basic settings of the elevator motor
    elevatorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12);

    coralTimeOfFlight = new TimeOfFlight(ShootConstants.kCoralTimeOfFlightPort);
    coralTimeOfFlight.setRangingMode(RangingMode.Short);

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

    constraints = new TrapezoidProfile.Constraints(
      ElevatorSubsystemConstants.maxVelocity,
      ElevatorSubsystemConstants.maxAcceleration);
    nextState = new TrapezoidProfile.State(0, 0);
    goalState = new TrapezoidProfile.State(0, 0);
    profile = new TrapezoidProfile(constraints);
  }

  /*
   * Move the elevator to the current target position. This method is called in the periodic method
   * of the subsystem.
   */
  public void moveToSetpoint() {
    // Update the state for motion towards target over the next 20ms
    nextState = profile.calculate(0.020, nextState, goalState);

    elevatorClosedLoopController.setReference(elevatorCurrentTarget, 
      ControlType.kMAXMotionPositionControl,
      ClosedLoopSlot.kSlot0, 
      feedforward.calculate(nextState.velocity)
      );
  }

  public double getDistance() {
    return coralTimeOfFlight.getRange();
  }

  /*
   * Command to set the elevator to a specific setpoint. The setpoint is defined in the Setpoint enum.
   * 
   * @param setpoint The setpoint to move the elevator to.
   * 
   * @return Command to set the elevator to a specific setpoint.
   */
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

          goalState = new TrapezoidProfile.State(elevatorCurrentTarget, 0);
        }).unless(this.getDistance() < ElevatorSubsystemConstants.intakeWidth);
  }

  /** 
   * Set the intake motor power in the range of [-1, 1]. 
   * 
   * @param power The power to set the intake motor to.
   */
  private void setShootPower(double power) {
    shootMotor.set(power);
  }

   /**
   * Command to run the intake motor. When the command is interrupted, e.g. the button is released,
   * the motor will stop.
   * 
   * @return Command to run the intake motor.
   */
  public Command runShootCommand() {
    return this.startEnd(
        () -> this.setShootPower(ShootConstants.shootPower), () -> this.setShootPower(0.0)
        );
  }

  /**
   * Command to run the intake motor. When the command is interrupted, e.g. the button is released,
   * the motor will stop.
   */
  public Command L1shootCommand() {
    return this.startEnd(
        () -> this.setShootPower(ShootConstants.L1shootPower), () -> this.setShootPower(0.0));
  }

  /**
   * Command to run the intake motor until the limit switch is no longer pressed (and the piece is out of the shooter).
   * When the limit switch is no longer pressed, the motor will stop.
   * 
   * @return Command to run the intake motor until the limit switch is no longer pressed.
   */
  public Command runShootCommandWithSwitch() {
    return this.runOnce(
      () -> this.setShootPower(ShootConstants.shootPower)
    ).unless(
      () -> coralIntakeLimitSwitch.get() == false
    ).until(  
      () -> coralIntakeLimitSwitch.get() == false
    ).andThen(
      () -> this.setShootPower(0.0)
    );
  }

  /**
   * Command to run the intake motor until the limit switch is pressed. When the limit switch is
   * pressed, the motor will stop.
   * 
   * @return Command to run the intake motor until the limit switch is pressed.
   */
  public Command intakeCommand() {
    return this.runOnce(
      () -> this.setShootPower(-ShootConstants.intakePower)
    ).until(
      () -> coralIntakeLimitSwitch.get() == true
    ).andThen(
      () -> this.setShootPower(0.0)
    );
  }

  /**
   * Command to reverse the intake motor. When the command is interrupted, e.g. the button is
   * released, the motor will stop.
   * 
   * @return Command to reverse the intake motor.
   */
  public Command reverseIntakeCommand() {
    return this.startEnd(
        () -> this.setShootPower(ShootConstants.intakePower), () -> this.setShootPower(0.0));
  }

  @Override
  public void periodic(){
    SmartDashboard.putNumber("Coral/Elevator/Target Position", elevatorCurrentTarget);
    SmartDashboard.putNumber("Coral/Elevator/Actual Position", elevatorEncoder.getPosition());
    moveToSetpoint();
  }
}

