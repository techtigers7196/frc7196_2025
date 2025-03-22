package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Algae;

public class AlgaeSubsystem extends SubsystemBase {

    public enum Setpoints {
        kalgae0,
        kalgae1,
        kalgae2;
    }
    
    private SparkMax algaeMotor = new SparkMax (Algae.kalgaeMotorCanId, MotorType.kBrushless);
    private final SparkMaxConfig algaeConfig = new SparkMaxConfig();
    private RelativeEncoder algaeEncoder = algaeMotor.getEncoder();

    //PID variables for manual PID
    private final double kP = 0.15;
    private final double kI = 0;
    private final double kD = 0;
    private final double kfeedforward = 0.003;

    //PID controller
    private final PIDController pid = new PIDController(kP, kI, kD);

    private double target = Algae.kalgae0;

    //private double target = 0;

    


    public AlgaeSubsystem(){     
        algaeConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12);

        algaeMotor.configure(
          algaeConfig,
          ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);
          algaeEncoder.setPosition(0);
     }

    //  public Command setSetpointCommand(Setpoint setpoint) {
    //     return this.runOnce(
    //         () -> {
    //             switch (setpoint) {
    //                 case kalgae0: 
    //                   target = Algae.kalgae0;
    //                   break;
    //                 case kalgae1:
    //                   target = Algae.kalgae1;
    //                   break;
    //                 case kalgae2:
    //                   target = Algae.kalgae2;
    //                   break;
    //             }
    //         }  
    //     );
    // }
    

    public Command setSetpointCommand(Setpoints setpoint) {
        return this.runOnce(
            () -> {
              switch (setpoint) {
                case kalgae0:
                    target = Algae.kalgae0;
                  break;
                case kalgae1:
                    target = Algae.kalgae1;
                  break;
                case kalgae2:
                  target = Algae.kalgae2;
                  break;
              }
            });
      }

    private void moveToSetpointPID(){
        double pidValue = pid.calculate(algaeEncoder.getPosition(),target);
        double power = kfeedforward+pidValue;
        algaeMotor.set(power);
    }

     /** Set the climb motor power in the range of [-1, 1]. */
    private void setAlgaePower(double power) {
        algaeMotor.set(power);
    }

    public Command algaeUpCommand(){
        return this.startEnd(
        () -> this.setAlgaePower(Algae.kalgaeUpPower), () -> this.setAlgaePower(0.0));
    }

    public Command algaeDownCommand(){
        return this.startEnd(
        () -> this.setAlgaePower(Algae.kalgaeDownPOwer), () -> this.setAlgaePower(0.0));
    }

    public void periodic() {
        SmartDashboard.putNumber("algae/Elevator/Target Position", target);
        SmartDashboard.putNumber("algae/Elevator/Actual Position", algaeEncoder.getPosition());
        moveToSetpointPID();
    }
}
