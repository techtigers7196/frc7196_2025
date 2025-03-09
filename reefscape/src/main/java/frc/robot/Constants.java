// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {
    public static final int kDriverControllerPort = 0;
    
    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  //Motor controller for elevator
  public static final class ElevatorSubsystemConstants {
    public static final int kelevatorMotorCanId = 9;

    public static final double kP = 0.2;  //0.2
    public static final double kI = 0;
    public static final double kD = 0.8;  //0.8
    public static final double kS = 0;  //0
    public static final double kG = 1.12;  //2.28
    public static final double kV = 3.84; //3.07
    public static final double kA = 0.14;  //0.41
    public static final double velocityFF = 0.002114164905;   //0.002114164905
    public static final double outputRangeMin = -1;
    public static final double outputRangeMax = 1;
    public static final double maxVelocity = 4200;
    public static final double maxAcceleration = 6000;
    public static final double allowedClosedLoopError = 0.5;
    public static final double intakeWidth = 300;

  }

  //Elevator heights variables 
  public static final class ElevatorSetpoints {
    public static final int kLevel1Intake = 0;
    public static final int kLevel2 = 5;
    public static final double kLevel3 = 12;
    public static final double kLevel4 = 23;
  }

  public static final class ShootConstants{
    public static final int kshootMotorCanId = 11;
    public static final int kCoralIntakeLimitSwitchPort = 0;
    public static final int kcoralTimeOfFlightPort = 0;
  //Shooting variables
    public static double shootPower = 0.2;
    public static double L1shootPower = 0.1;
    public static double intakePower = 0.2;
    // public static double shootPrevent = -0.3;
    // public static double feedPower = 0.3;
  }

  public static final class OIConstants {
    public static final double kTriggerButtonThreshold = 0.1;
  }

  public static final class AlignmentConstants{
    public static final double kPSwerveAlignX = 0.01;
    public static final double kPSwerveAlignZ = 0.01;
    public static final double kPSwerveAlignTheta = 0.01;

    public static final double kPSwerveAlign2DStrafe = 0.075;
    public static final double kPSwerveAlign2DForward = 0.1; //.15
    public static final double feedforward = 0.01;
    public static final double ktxTargetRight = -16;
    public static final double ktyTarget = -14;
    public static final double ktxTargetLeft = 16;
  }
     
  public static final class Algea{
    public static final int kalgeaMotorCanId = 12;
    public static final double kalgeaUpPower = 0.1;  //?
    public static final double kalgeaDownPOwer = -0.1; //?
  }

  public static final class Climb{
    public static final int kclimbMotorCanId = 13;
    public static final double kclimbPower = 0.2;
    public static final double kclimbDownPower = -0.2;
  }
}