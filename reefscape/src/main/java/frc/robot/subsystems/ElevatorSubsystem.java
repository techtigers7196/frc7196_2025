package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

//Libraries for motor controllers
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig; 


import edu.wpi.first.math.controller.PIDController;

//Libraries for Elevator
import frc.robot.Constants.ElevatorSubsystemConstants;
import frc.robot.Constants.ElevatorSetpoints;
import frc.robot.Constants.ShootConstants;

//Smart Dashboard library 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


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
//public static final SparkMaxConfig shootConfig = new SparkMaxConfig();

    //shooting controllers 
private SparkMax shootMotor = new SparkMax(ShootConstants.kshootMotorCanId,MotorType.kBrushless);
//private RelativeEncoder shootEncoder = shootMotor.getEncoder();

    // public void shoot() {
    //     this.shootMotor.set(ShootConstants.shootPower);
        // this.shootMotor.set(ShootConstants.shootPrevent);
        // this.shootMotor.set(ShootConstants.feedPower);
    
// }

//sets the PID VALUES
public ElevatorSubsystem(){

     // Configure basic settings of the elevator motor
      elevatorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12);

      /*
       * Configure the reverse limit switch for the elevator. By enabling the limit switch, this
       * will prevent any actuation of the elevator in the reverse direction if the limit switch is
       * pressed.
       */
    //   elevatorConfig
    //       .limitSwitch
    //       .reverseLimitSwitchEnabled(true)
    //       .reverseLimitSwitchType(Type.kNormallyOpen);

    //Function to intake a note

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      elevatorConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .p(0.2)
          .d(0.08)
          .velocityFF(0.002114164905)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(4200)
          .maxAcceleration(6000)
          .allowedClosedLoopError(0.5);

          elevatorMotor.configure(
            elevatorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

            // Configure basic settings of the intake motor
            //shootConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);

             // Zero elevator encoders on initialization
            elevatorEncoder.setPosition(0);
    }

     /** Set the intake motor power in the range of [-1, 1]. */
  private void setShootPower(double power) {
    shootMotor.set(power);
  }

  // Member variables for subsystem state management
//   private boolean wasResetByButton = false;
//   private boolean wasResetByLimit = false;
  private double elevatorCurrentTarget = ElevatorSetpoints.kLevel1Intake;

  public Command moveToSetpoint() {
    return this.startEnd(
        () -> elevatorClosedLoopController.setReference(
        elevatorCurrentTarget, ControlType.kMAXMotionPositionControl), () -> elevatorClosedLoopController.setReference(
        elevatorCurrentTarget, ControlType.kMAXMotionPositionControl));
  }

// public void setelevatorPosition(double position) {
//     elevatorPID.setReference(position, SparkMax.ControlType.kPosition);
// }


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
        });
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

  @Override
  public void periodic(){
    SmartDashboard.putNumber("Coral/Elevator/Target Position", elevatorCurrentTarget);
    SmartDashboard.putNumber("Coral/Elevator/Actual Position", elevatorEncoder.getPosition());
  }
}

