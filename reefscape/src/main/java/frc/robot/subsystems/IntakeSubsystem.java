package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorSubsystemConstants;
import frc.robot.Constants.ShootConstants;

public class IntakeSubsystem extends SubsystemBase {

    private TimeOfFlight coralTimeOfFlight;

    public static final SparkMaxConfig shootConfig = new SparkMaxConfig();
    private SparkMax shootMotor = new SparkMax(ShootConstants.kshootMotorCanId,MotorType.kBrushless);

    private DigitalInput coralIntakeLimitSwitch = new DigitalInput(ShootConstants.kCoralIntakeLimitSwitchPort);

    public IntakeSubsystem(TimeOfFlight timeOfFlight) {

        coralTimeOfFlight = timeOfFlight;

        shootConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(30).voltageCompensation(12);

        shootMotor.configure(
            shootConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    public double getDistance(){
        return coralTimeOfFlight.getRange();
      }
    

    public boolean hasCoralInElevatorPath() {
        if(this.getDistance() < ElevatorSubsystemConstants.intakeWidth){
        return true;
        }

        return false;
    }

    private void setShootPower(double power) {
        shootMotor.set(power);
      }

      public boolean isCoralLoaded() {
        // return !this.coralIntakeLimitSwitch.get() && !this.hasCoralInElevatorPath();
        return !this.coralIntakeLimitSwitch.get() && !this.hasCoralInElevatorPath();
      }

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

      public Command intakeCommandWithSwitch(){
    return this.run(
      ()-> this.setShootPower(ShootConstants.intakePower)
    ).until(
      () -> this.isCoralLoaded()
    ).andThen(
      ()-> this.setShootPower(0)
    );
  }

  public Command runShootCommand() {
    return this.startEnd(
        () -> this.setShootPower(ShootConstants.shootPower), () -> this.setShootPower(0.0));
  }

  public Command reverseIntakeCommand() {
    return this.startEnd(
        () -> this.setShootPower(-ShootConstants.intakePower), () -> this.setShootPower(0.0));
  }

  public Command intakeCommand() {
    return this.startEnd(
        () -> this.setShootPower(ShootConstants.intakePower), () -> this.setShootPower(0.0));
  }

  public void periodic(){
        SmartDashboard.putBoolean("Coral/Shooter/IsLoaded", this.isCoralLoaded());

  }

}
