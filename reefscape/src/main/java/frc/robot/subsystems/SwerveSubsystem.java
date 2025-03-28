// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.reduxrobotics.frames.DoubleFrame.DoubleToType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import frc.robot.Constants.AlignmentConstants;

public class SwerveSubsystem extends SubsystemBase {
  
  private final SwerveDrive  swerveDrive;
  private double strafeError;
  private double forwardError;
  private int noVisionCount = 0;
  private int goalCount = 0;
  
  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem(){
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
    try
    {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Constants.MAX_SPEED);
      //swerveDrive.getGyro().setOffset(swerveDrive.getGyroRotation3d().unaryMinus());
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();

      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose, // Robot pose supplier
          this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speeds, feedforwards) -> swerveDrive.setChassisSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                  new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                  new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
          ),
          config, // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this // Reference to this subsystem to set requirements
      );
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
  }

  /**
   * Angular velocity drive command.
   *
   * @return a command
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY)
  {
    return this.run(() -> {
      // Make the robot move
      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                                                translationY.getAsDouble()), 0.7);
        
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(), headingX.getAsDouble(), headingY.getAsDouble(), swerveDrive.getOdometryHeading().getRadians(), swerveDrive.getMaximumChassisVelocity()));
    });
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveRobotOriented(Supplier<ChassisSpeeds> velocity) {
    return this.run(() -> {
      swerveDrive.drive(velocity.get());
    });
  }

  public Command driveFieldOrientedCommand(Supplier<ChassisSpeeds> velocity) {
    return this.run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  public Command moveToPosition2DCommand (double txTarget, double tyTarget, VisionSubsystem vision, boolean right){  
    return this.run(()->{
      double[] xya;
      //if aligning to the right we need to use the left LL and viceversa
      if(right){
        xya = vision.getXYA();
      } else {
        xya= vision.getXYARight();
      }
      //read values from Limelight
      double tx = xya[0];
      double ty = xya[1];
      double ta = xya[2];

      if(ta !=0.0){
        //We have vision again so reset noVisionCOunt
        noVisionCount = 0;
        strafeError = tx - txTarget;
        forwardError = ty + tyTarget;

        double forward = -AlignmentConstants.kPForward * forwardError;
        double strafe = -AlignmentConstants.kPStrafe * strafeError;

        if(Math.abs(strafe) < 1) strafe = strafe + Math.signum(strafe)*AlignmentConstants.feedforward;
        if(Math.abs(forward) < 1)forward = forward + Math.signum(forward)*AlignmentConstants.feedforward;

        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(forward, strafe, 0);
        swerveDrive.drive(desiredSpeeds);
      } else {
        //Every count of noVisionCount should equal 20ms
        //We use this to tell if we have lost vision for more than a certain tolerance
        noVisionCount++;
      }
    }).until(
      () -> {
        boolean reachedGoalState = false;
        // Did we get into our goal state?
        if (Math.abs(strafeError) < AlignmentConstants.strafeTolerance && Math.abs(forwardError) < AlignmentConstants.forwardTolerance) {
          //Debounce to accomodate for jitter in limelight readings when reaching goal state
          goalCount++;
          if(goalCount >= AlignmentConstants.kGoalCountMax) {
            reachedGoalState = true;
          } 
        } else {
          goalCount = 0;
          //Did we lose limelight readings for X period of time?
          if(noVisionCount >= AlignmentConstants.kNoVisionMax) {
            reachedGoalState = true;
          }
        }
        return reachedGoalState;
      }
    );
  }

  public ChassisSpeeds calcStrafeChassisSpeeds(double[] xya) {
      double tx = xya[0];
      double ta = xya[2];

      ChassisSpeeds desiredSpeeds = new ChassisSpeeds(0,0,0);

      if(ta !=0.0){
        strafeError = tx;
        double strafe = -AlignmentConstants.kPStrafe * strafeError;

        strafe = strafe + Math.signum(strafe)*AlignmentConstants.feedforward;

        desiredSpeeds = new ChassisSpeeds(0, strafe, 0);
      }

      return desiredSpeeds;
  }

  public Command moveToTag2DCommand (VisionSubsystem vision, boolean right){
    return this.alignToTag2DCommand(vision, right).andThen(
      this.moveToPosition2DCommand(AlignmentConstants.kXTarget, AlignmentConstants.kYTarget, vision, right
    ));
  }

  public Command alignToTag2DCommand (VisionSubsystem vision, boolean right){
    return this.run(()->{
      //read values from the right Limelight
      double[] xya = vision.getXYA();
      if(right) {
        xya = vision.getXYARight();
      }
    
      swerveDrive.drive(this.calcStrafeChassisSpeeds(xya));
    }).until(
      () -> Math.abs(strafeError) < AlignmentConstants.strafeTolerance
    );
  }

  public Command moveToTagCommand(double yOffset, VisionSubsystem vision){
    return this.run(()->{
      double[] pose = vision.getCameraPose();

      if (vision.hasTarget()){
        double dz = pose[2];
        double dx = yOffset - pose[0];

        ChassisSpeeds desiresSpeeds = new ChassisSpeeds(-AlignmentConstants.kPSwerveAlignZ * dz,-AlignmentConstants.kPSwerveAlignX * dx,0);
        swerveDrive.drive(desiresSpeeds);
      }
    });
  }

  //set the gyro to zero
  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  /**
   * Gets the swerve drive object.
   *
   * @return {@link SwerveDrive}
   */
  public SwerveDrive getSwerveDrive()
  {
    return swerveDrive;
  }

   /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

   /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Strafe", strafeError);
    SmartDashboard.putNumber("Forward", forwardError);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
