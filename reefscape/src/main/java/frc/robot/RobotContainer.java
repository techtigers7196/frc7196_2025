// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.AlgaeSubsystem.Setpoints;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Setpoint;
import swervelib.SwerveInputStream;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ShootConstants;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.Constants.AlignmentConstants;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * s ubsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase  = new SwerveSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final VisionSubsystem visionSubsystem = new VisionSubsystem();
    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
    private final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();
  
    final CommandXboxController driverXbox = new CommandXboxController(0);
    final CommandXboxController supportXbox = new CommandXboxController(1);
  
    private final SendableChooser<Command> autoChooser;
  
    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
     */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                  () -> driverXbox.getLeftY()*-1,
                                                                  () -> driverXbox.getLeftX()*-1)
                                                              .withControllerRotationAxis(() -> driverXbox.getRightX()*-1)
                                                              .deadband(OperatorConstants.DEADBAND)
                                                              .scaleTranslation(0.8)
                                                              .allianceRelativeControl(false);
  
    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOrientedCommand(driveAngularVelocity);

    Command driveRobotOrientedAngularVelocity = drivebase.driveRobotOriented(driveAngularVelocity);
  
    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(driverXbox.getLeftY() * -1, OperatorConstants.DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getLeftX() * -1, OperatorConstants.LEFT_Y_DEADBAND), 
      () -> driverXbox.getRightX() *-1,
      () -> driverXbox.getRightY()*-1);
  
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
      // Configure the trigger bindings
      configureBindings();

      NamedCommands.registerCommand("moveToL3", elevatorSubsystem.setSetpointCommand(Setpoint.kLevel3));
      NamedCommands.registerCommand("moveToL4", elevatorSubsystem.setSetpointCommand(Setpoint.kLevel4));
      NamedCommands.registerCommand("moveToL1", elevatorSubsystem.setSetpointCommand(Setpoint.kLevel1Intake));
      NamedCommands.registerCommand("shoot", elevatorSubsystem.runShootCommandWithSwitch());
      NamedCommands.registerCommand("intake", elevatorSubsystem.intakeCommandWithSwitch());
      NamedCommands.registerCommand("AlignRight", drivebase.moveToTag2DCommand(visionSubsystem, true));
      
      // Build an auto chooser. This will use Commands.none() as the default option.
      autoChooser = AutoBuilder.buildAutoChooser();
  
      // Another option that allows you to specify the default auto by its name
      SmartDashboard.putData("Auto Chooser", autoChooser);    
    }
  
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */


    private void configureBindings() {
      //Set default drive style
      drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
  
      //Update drive style to angular velocity
      driverXbox.a().onTrue(driveFieldOrientedAngularVelocity);
  
      //Update drive style to direct angle
      driverXbox.b().onTrue(driveFieldOrientedDirectAngle);

      //Update drive style to robot oriented angular velocity
      driverXbox.x().onTrue(driveRobotOrientedAngularVelocity);
  
      //sets the gyro to zero
      driverXbox.y().onTrue(Commands.runOnce(drivebase::zeroGyro));
  
      //X button sets the elevator to level 1 
      supportXbox.a().onTrue(elevatorSubsystem.setSetpointCommand(Setpoint.kLevel1Intake));
  
      //B button sets the elevator to Level 2
      supportXbox.b().onTrue(elevatorSubsystem.setSetpointCommand(Setpoint.kLevel2));
  
      //X button sets the elevator to Level 3
      supportXbox.x().onTrue(elevatorSubsystem.setSetpointCommand(Setpoint.kLevel3));
  
      //Y button sets the elevator to Level 4
      supportXbox.y().onTrue(elevatorSubsystem.setSetpointCommand(Setpoint.kLevel4));
    
      //shoot the coral L2-L4
      driverXbox.rightTrigger(OIConstants.kTriggerButtonThreshold)
      .onTrue(drivebase.alignToTag2DCommand(visionSubsystem, true));

      //shoot the coral L2-L4
      supportXbox.rightTrigger(OIConstants.kTriggerButtonThreshold)
      .whileTrue(elevatorSubsystem.runShootCommand());

      //shoot the coral into L1
      driverXbox.leftTrigger(OIConstants.kTriggerButtonThreshold)
      .onTrue(drivebase.alignToTag2DCommand(visionSubsystem, false));

      //shoot the coral into L1
      supportXbox.leftTrigger(OIConstants.kTriggerButtonThreshold)
      .onTrue(elevatorSubsystem.intakeCommandWithSwitch());

      //intakes the coral a  bit 
      supportXbox.leftBumper().whileTrue(elevatorSubsystem.intakeCommand());

      //intakes the coral a bit
      supportXbox.rightBumper().whileTrue(elevatorSubsystem.reverseIntakeCommand());

      driverXbox.povRight().onTrue(drivebase.moveToTag2DCommand(visionSubsystem, true));

      driverXbox.povLeft().onTrue(drivebase.moveToTag2DCommand(visionSubsystem, false));

      supportXbox.povUp().whileTrue(climbSubsystem.climbCommand());

      supportXbox.povDown().whileTrue(climbSubsystem.climbDownCommand());

      // supportXbox.povLeft().whileTrue(algaeSubsystem.algaeUpCommand());

      // supportXbox.povRight().whileTrue(algaeSubsystem.algaeDownCommand());

      //the inatkes retracts for endgame
      supportXbox.back().whileTrue(elevatorSubsystem.retractIntake());

      supportXbox.start().whileTrue(elevatorSubsystem.unretractIntake());

      //left bumper moves the algae arm  to starting position
      driverXbox.leftBumper().onTrue(algaeSubsystem.setSetpointCommand(Setpoints.kalgae1));
  
      //right bumper moves the algae arm to second setpoint
      driverXbox.rightBumper().onTrue(algaeSubsystem.setSetpointCommand(Setpoints.kalgae2));
  
      //right bumper moves the algae arm to third/highest setpoint
      driverXbox.back().onTrue(algaeSubsystem.setSetpointCommand(Setpoints.kalgae0));

      driverXbox.povUp().whileTrue(drivebase.moveToTagCommand(0.15,visionSubsystem));
    }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}