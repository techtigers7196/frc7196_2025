package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Climb;
import frc.robot.Constants.winch;

public class ClimbSubsystem extends SubsystemBase{

     private SparkMax climbMotor = new SparkMax (Climb.kclimbMotorCanId, MotorType.kBrushless);
     private final SparkMaxConfig climbConfig = new SparkMaxConfig();

     private SparkMax winchMotor = new SparkMax (winch.kwinchMotorCanId, MotorType.kBrushed);

     public ClimbSubsystem(){
        
        climbConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12);

        climbMotor.configure(
          climbConfig,
          ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);

     }

    /** Set the climb motor power in the range of [-1, 1]. */
    private void setClimbPower(double power) {
        climbMotor.set(power);
    }

    public Command retractIntake() {
        return this.startEnd(
        () -> winchMotor.set(-0.2), () -> winchMotor.set(0.0));
    }

    public Command unretractIntake() {
        return this.startEnd(
        () -> winchMotor.set(0.1), () -> winchMotor.set(0.0));
    }

    public Command climbCommand(){
        return this.startEnd(
        () -> this.setClimbPower(Climb.kclimbPower), () -> this.setClimbPower(0.0));
    }

    public Command climbDownCommand(){
        return this.startEnd(
        () -> this.setClimbPower(Climb.kclimbDownPower), () -> this.setClimbPower(0.0));
    }
}
