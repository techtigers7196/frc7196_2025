package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Algea;

public class AlgeaSubsystem extends SubsystemBase {
    
    private SparkMax algeaMotor = new SparkMax (Algea.kalgeaMotorCanId, MotorType.kBrushless);
     private final SparkMaxConfig algeaConfig = new SparkMaxConfig();

     public AlgeaSubsystem(){
        
        algeaConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12);

        algeaMotor.configure(
          algeaConfig,
          ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);

     }

     /** Set the climb motor power in the range of [-1, 1]. */
    private void setAlgeaPower(double power) {
        algeaMotor.set(power);
    }

    public Command algeaUpCommand(){
        return this.startEnd(
        () -> this.setAlgeaPower(Algea.kalgeaUpPower), () -> this.setAlgeaPower(0.0));
    }

    public Command algeaDownCommand(){
        return this.startEnd(
        () -> this.setAlgeaPower(Algea.kalgeaDownPOwer), () -> this.setAlgeaPower(0.0));
    }
}
