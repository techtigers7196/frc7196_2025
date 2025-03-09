package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climb;

public class ClimbSubsystem extends SubsystemBase{

     private SparkMax climbMotor = new SparkMax (Climb.kclimbMotorCanId, MotorType.kBrushless);
     private final SparkMaxConfig climbConfig = new SparkMaxConfig();

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

    public Command climbCommand(){
        return this.startEnd(
        () -> this.setClimbPower(Climb.kclimbPower), () -> this.setClimbPower(0.0));
    }

    public Command climbDownCommand(){
        return this.startEnd(
        () -> this.setClimbPower(Climb.kclimbDownPower), () -> this.setClimbPower(0.0));
    }
}
