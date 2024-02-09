// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Function;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.drivetrain.SwerveModuleConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.subsystems.leds.patterns.FadeLEDPattern;
import frc.robot.subsystems.leds.patterns.LEDPattern;
import frc.robot.subsystems.leds.patterns.MorseCodePattern;
import frc.robot.subsystems.leds.patterns.RainbowPattern;
import frc.robot.subsystems.leds.patterns.SineLEDPattern;
import frc.robot.subsystems.leds.patterns.SolidLEDPattern;
import frc.robot.subsystems.leds.patterns.TestColorPattern;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
 
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kManipulatorControllerPort = 1; 
  }
  public static class DrivetrainConstants {
    // Swerve IDs
    public static SwerveModuleConfig frontLeft = new SwerveModuleConfig("FrontLeft", 1, 2, 12, 0.44921875, true, true, SensorDirectionValue.CounterClockwise_Positive); 
    public static SwerveModuleConfig frontRight = new SwerveModuleConfig("FrontRight", 7, 8, 9, -0.160889, true, true, SensorDirectionValue.CounterClockwise_Positive); 
    public static SwerveModuleConfig backLeft = new SwerveModuleConfig("BackLeft", 3, 4, 10, 0.216797, true, true, SensorDirectionValue.CounterClockwise_Positive); 
    public static SwerveModuleConfig backRight = new SwerveModuleConfig("BackRight", 5, 6, 11, 0.167236, true, true, SensorDirectionValue.CounterClockwise_Positive); 


    // Gearing & Conversions
    public static final double kGearRatio = 6.12; // driving gear ratio of each swerve module
    public static final double kWheelRadiusInches = 2; // radius of the wheels
    public static final double kMetersPerRot = Units.inchesToMeters(2 * Math.PI * kWheelRadiusInches / kGearRatio); // calculate the position conversion factor of the swerve drive encoder
    public static final double kMetersPerSecondPerRPM = kMetersPerRot / 60; // calculate the velocity conversion factor of the swerve drive encoder

    public static final double kRotateGearRatio = 1; // gear ratio of the turn encoder (will be 1 as long as we use CANCoders on the output shaft)
    public static final double kDegreesPerRot = 360 / kRotateGearRatio; // position conversion factor of the turn encoder (converts from rotations to degrees)
    public static final double kDegreesPerSecondPerRPM = kDegreesPerRot / 60; // velocity conversion factor of the turn encoder 

    // Drivebase
    public static final double kTrackWidthMeters = Units.inchesToMeters(20.75); // horizontal dist between wheels
    public static final double kWheelBaseMeters = Units.inchesToMeters(22.75); // vertical dist between wheels

    public static final double kDriveBaseRadius = Math.hypot(kTrackWidthMeters, kWheelBaseMeters) / 2; 

    // Kinematics
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      // wheel locations relative to the center of the robot
      new Translation2d(kTrackWidthMeters / 2, kWheelBaseMeters / 2), // front left
      new Translation2d(kTrackWidthMeters / 2, -kWheelBaseMeters / 2), // front right
      new Translation2d(-kTrackWidthMeters / 2, kWheelBaseMeters / 2), // back left
      new Translation2d(-kTrackWidthMeters / 2, -kWheelBaseMeters / 2) // back right
    ); 

    // copied speeds (https://github.com/SwerveDriveSpecialties/swerve-template-2021-unmaintained/blob/master/src/main/java/frc/robot/subsystems/DrivetrainSubsystem.java)
    public static final double kMaxAttainableModuleSpeedMetersPerSecond = 5880.0 / 60.0 / kGearRatio *
    2 * Units.inchesToMeters(kWheelRadiusInches) * Math.PI; // max attainable speed for each drive motor
    public static final double kMaxAttainableRotationRadPerSecond = kMaxAttainableModuleSpeedMetersPerSecond /
    Math.hypot(kTrackWidthMeters / 2.0, kWheelBaseMeters / 2.0); // max rotation of robot
    
    // TODO: tune these
    public static double kMaxSpeedMetersPerSecond = 2.0; // max velocity (no turning) of robot; may tune to be a fraction of the attainable module speed
    public static double kMaxSlowSpeedMetersPerSecond = 1.0; 
    public static final double kMaxAccelerationMetersPerSecondSquared = kMaxSpeedMetersPerSecond / 0.2; // max acceleration of robot (accelerate to max speed in 1 second)
    public static double kMaxRotationRadPerSecond = 3.14; // max rotation speed of the robot
    public static final double kMaxSlowRotationRadPerSecond = Math.PI / 2; 
    public static final double kMaxRotationAccelerationRadPerSecondSquared = kMaxRotationRadPerSecond / 0.2; // max angular acceleration of robot

    // feedforward values (NO NEED to tune these)
    public static final double ksVolts = 0; 
    public static final double kvVoltSecondsPerMeter = 0; 
    public static final double kaVoltSecondsSquaredPerMeter = 0; 

    // drive speed PID values for a swerve module
    public static final double kModuleDrive_P = 0.0006890099939482752; 
    public static final double kModuleDrive_I = 0; 
    public static final double kModuleDrive_D = 0; 
    public static final double kModuleDrive_FF = 0.2;

    // found from sysid for one of the turn modules or tune by yourself
    // turn PID values for a swerve module
    public static final double kModuleTurn_P = 0.0081; 
    public static final double kModuleTurn_I = 0; 
    public static final double kModuleTurn_D = 0.00032; 

    // turn in place PID for the whole robot
    public static final double kTurn_P = 0.054; 
    public static final double kTurn_I = 0; 
    public static final double kTurn_D = 0.001; 
    public static final double kTurn_FF = 0; 
    public static final double kTurnErrorThreshold = 2.0; 
    public static final double kTurnVelocityThreshold = 0; 

    // TODO: tune these but it should be fine
    // current limits for each motor
    public static final int kDriveCurrentLimit = 40; 
    public static final double kDriveRampRate = 0.25; 
     
    public static final int kTurnCurrentLimit = 20; 
    public static final double kTurnRampRate = 0.25;

    // max acceleration/deceleration of each motor (used for high CG robots)
    public static final double kForwardSlewRate = kMaxAccelerationMetersPerSecondSquared; 
    public static final double kStrafeSlewRate = kMaxAccelerationMetersPerSecondSquared; 
    public static final double kTurnSlewRate = kMaxRotationAccelerationRadPerSecondSquared; 
  }

  public static class Trajectory {
    // translational PID of robot for trajectory use
    public static final double kDrive_P = 2.25; 
    public static final double kDrive_I = 0; 
    public static final double kDrive_D = 0;

    // angular PID (same as turn pid)
    public static final double kOmega_P = 4; 
    public static final double kOmega_I = 0; 
    public static final double kOmega_D = 0.0001; 

    // max velocity & acceleration robot can go n following a trajectory
    public static final double kMaxVelocityMetersPerSecond = 1; 
    public static final double kMaxAccelerationMetersPerSecondSquared = 1; 

    public static final double kMaxAngularVelocityRadiansPerSecond = Units.degreesToRadians(120); 
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Units.degreesToRadians(120); 
  }

  public static class IntakeConstants {
    public static int kIntakeMotorShootTopId = 15;
    public static int kIntakeMotorShootBottomId = 16;
    public static int kOuttakeMotorSuckId = 17;
    

    public static boolean kShootTopIntakeInverted = false;
    public static int kShootTopMotorlimit = 40;

    public static boolean kShootBottomIntakeInverted = false; 
    public static int kShootBottomMotorlimit = 40;

    public static final boolean kSuckOuttakeInverted = false;
    public static final int kSuckOuttakeMotorLimit = 40;

    public static final double kShootVelocityConversionFactor = 1; 
    public static final double kIntakeVelocityConversionFactor = 1; 

    public static final int kBeamBreakId = 0;

    public static final double kSuckerIntakeSpeed = 0.5;

    public static final double kShootSpeed = 1; 
    public static final double kShootSuckerSpeed = 1; 
    public static final double kShootRevTime = 0.5; 
    public static final double kShootWaitTime = 0.5; 

    public static final double kAmpOuttakeSpeed = 0.5;
    public static final int kSuckerManualSpeed = 0; 
  }

  public class ArmConstants {
    public static final int kLimitSwitchId = 1;
    public static final int kArmMotorRightId = 14;
    public static final int kArmMotorLeftId = 13;

    public static final boolean kRightMotorInverted = false;
    public static final boolean kLeftMotorInverted = false; 

    public static final int kMotorCurrentLimit = 40;
    
    public static final float kForwardSoftLimit = 100;
    public static final double kReverseSoftLimit = 0; 

    public static final double kPositionConversionFactor = 360;

    // velocity = position / 60
    public static final double kVelocityConversionFactor = 360 / 60.0; 
    public static final double kTolerance = 3;

    public static final double kGroundPosition = 0;
    public static final double kTravelPosition = 20;
    public static final double kAmpPosition = 90;
    public static final double kSpeakerPosition = 10;

    public static final double kTravelSpeed = 0.5;


    // TODO: tune arm 
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    // motion profile
    public static final double kMaxArmVelocity = 0;
    public static final double kMaxArmAcceleration = 0; 

    // custom feedforward
    // https://www.desmos.com/calculator/3mcdsjeubz
    // public static final double kGravityFF = 0;
    // public static final double kSpringFF = 0;

     
    // calculate using reca.lc
    // CoM distance: 21.77 in
    // Arm mass: 20.755 lbs
    
    public static final double kS = 0; 
    public static final double kG = 0; // 0.79 V
    public static final double kV = 0; // 1.95 V*s/rad
    public static final double kA = 0; // 0.06 V*s^2/rad
    

  }

  public static class HangConstants {
    public static final int kHangRightMotorID = 19;
    public static final int kHangLeftMotorID = 18;

    public static final boolean kRightMotorInverted = false;
    public static final boolean kLeftMotorInverted = false;

    public static final int kHangMotorLimit = 40;

    public static final double kPositionConversionFactor = 1;
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60;

    public static final float kFowardHangSoftLimit = 0;
    public static final float kReverseHangSoftLimit = 0;

    public static final double kHangSpeed = 0.25;
  }

  public static class Vision {
    public static final double kTranslationStdDevCoefficient = 0.35;
    public static final double kRotationStdDevCoefficient = 1;
    public static final AprilTagFieldLayout kAprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(); 
    public static final double limelightZHeight = 0; // TODO: Fix this
    public static final double kMaxAccuracyRange = 1000;
    public static final Pose2d kRedAllianceSpeaker = new Pose2d(0, 5.54, new Rotation2d());
    public static final Pose2d kBlueAllianceSpeaker = new Pose2d(16.5, 5.54, new Rotation2d());

    // TODO: tune
    public static final Function<Double, Double> distanceToArmAngle = (dist) -> 0.0; 

    static {
      kAprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    }
  }
  public static final class LEDs {
    public static final Color kDefaultColor = Color.kDarkBlue;
    public static final Color kCobaltBlue = new Color("#0047AB");
    public static final Color kGold = new Color(142, 66, 0); // 142.17, 66.44

    public static final int kLedPort = 6; 
    public static final int kLedLength = 40; 

    public static final int kLed1Start = 0; 
    public static final int kLed1End = 10; 
    public static final int kLed2Start = 10; 
    public static final int kLed2End = 20;
    public static final int kLed3Start = 20;
    public static final int kLed3End = 30;
    public static final int kLed4Start = 30;
    public static final int kLed4End = 40;

    public static final class Patterns {
      public static final LEDPattern kDefault = new SolidLEDPattern(LEDs.kDefaultColor);
      public static final LEDPattern kDisabled = new FadeLEDPattern(4, LEDs.kDefaultColor, kGold);
      public static final LEDPattern kCube = new SolidLEDPattern(Color.kPurple);
      public static final LEDPattern kCone = new SolidLEDPattern(kGold);
      public static final LEDPattern kDead = new MorseCodePattern(Color.kRed, kCobaltBlue, "dead");
      public static final LEDPattern kDeadAlternate = new FadeLEDPattern(1, Color.kRed, Color.kBlack);
      public static final LEDPattern kBalanceFinished = new RainbowPattern(0.5);
      public static final LEDPattern kAllianceRed = new SolidLEDPattern(Color.kRed);
      public static final LEDPattern kAllianceBlue = new SolidLEDPattern(Color.kBlue);
      public static final LEDPattern kEnabled = new SineLEDPattern(1, kGold, kCobaltBlue, 8);
      public static final LEDPattern kIdle= new FadeLEDPattern(2, Color.kRed, kGold);
      public static final LEDPattern kMoving = new FadeLEDPattern(1,kGold, Color.kWhite);
      public static final LEDPattern kFail = new FadeLEDPattern(1,Color.kRed, kGold);
      public static final LEDPattern kIntake = new FadeLEDPattern(1,kCobaltBlue, Color.kGreen);
      public static final LEDPattern kShootAnywhere = new SolidLEDPattern(kCobaltBlue);
      public static final LEDPattern kArmMoving = new SolidLEDPattern(Color.kOrange);
      public static final LEDPattern kArmAtAmp = new SolidLEDPattern(Color.kPink);
      public static final LEDPattern kArmAtSpeaker = new SolidLEDPattern(kGold);
      public static final LEDPattern kArmAtGround = new SolidLEDPattern(Color.kDarkGreen);
      // public static final LEDPattern kShootAmp = new SolidLEDPattern(Color.kBlue);
      // public static final LEDPattern kShootSpeaker = new SolidLEDPattern(Color.kBlue);
      public static final LEDPattern kArmCustom = new SolidLEDPattern(Color.kSeaGreen);
      public static final LEDPattern kHangActive = new SineLEDPattern(2, Color.kPurple, Color.kBlack, 5);
      public static final LEDPattern kBeamHit = new SolidLEDPattern(Color.kGreen, 3);
      public static final LEDPattern kAuto = new FadeLEDPattern(1, Color.kPurple, Color.kBlack);
      public static final LEDPattern kTestColor = new TestColorPattern();
    }

  }
  public static final class PathFollower {
    public static final Pose2d speakerBlue1 = new Pose2d(0.66, 6.60, Rotation2d.fromDegrees(60));
    public static final Pose2d speakerBlue2 = new Pose2d(1.25, 5.48, Rotation2d.fromDegrees(0));
    public static final Pose2d speakerBlue3 = new Pose2d(0.69, 4.49, Rotation2d.fromDegrees(-60));
    public static final Pose2d speakerRed1 = new Pose2d(15.83, 6.58, Rotation2d.fromDegrees(120));
    public static final Pose2d speakerRed2 = new Pose2d(15.29, 5.48, Rotation2d.fromDegrees(180));
    public static final Pose2d speakerRed3 = new Pose2d(15.83, 4.51, Rotation2d.fromDegrees(-120));
    public static final Pose2d ampBlue = new Pose2d(1.96, 7.75, Rotation2d.fromDegrees(-90));
    public static final Pose2d ampRed = new Pose2d(14.62, 7.75, Rotation2d.fromDegrees(-90));
  }
  
  
  
}
