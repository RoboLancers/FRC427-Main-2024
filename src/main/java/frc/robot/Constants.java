// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Function;
import java.util.function.BooleanSupplier; 

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import  edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.SwerveModuleConfig;
import frc.robot.subsystems.intake.Intake;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.subsystems.leds.patterns.ConditionalPattern;
import frc.robot.subsystems.leds.patterns.FadeLEDPattern;
import frc.robot.subsystems.leds.patterns.LEDPattern;
import frc.robot.subsystems.leds.patterns.RainbowPattern;
import frc.robot.subsystems.leds.patterns.SineLEDPattern;
import frc.robot.subsystems.leds.patterns.SolidLEDPattern;
import frc.robot.subsystems.leds.patterns.StrobePattern;
import frc.robot.subsystems.leds.patterns.TestColorPattern;
import frc.robot.util.quad.OrderedPair;

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
    public static SwerveModuleConfig frontLeft = new SwerveModuleConfig("FrontLeft", 8, 7, 12, 0.44921875, true, true, SensorDirectionValue.CounterClockwise_Positive); 
    public static SwerveModuleConfig frontRight = new SwerveModuleConfig("FrontRight", 2, 1, 9, -0.160889, true, true, SensorDirectionValue.CounterClockwise_Positive); 
    public static SwerveModuleConfig backLeft = new SwerveModuleConfig("BackLeft", 6, 5, 10, 0.216797, true, true, SensorDirectionValue.CounterClockwise_Positive); 
    public static SwerveModuleConfig backRight = new SwerveModuleConfig("BackRight", 4, 3, 11, 0.167236, true, true, SensorDirectionValue.CounterClockwise_Positive); 


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
    
    public static double kMaxSpeedMetersPerSecond = 1; // max velocity (no turning) of robot; may tune to be a fraction of the attainable module speed
    public static double kMaxSlowSpeedMetersPerSecond = 1.0; 
    public static final double kMaxAccelerationMetersPerSecondSquared = kMaxSpeedMetersPerSecond / 0.2; // max acceleration of robot (accelerate to max speed in 1 second)
    public static double kMaxRotationRadPerSecond = 3.00; // 3.00; // max rotation speed of the robot
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
    public static final double kModuleDrive_FF = 0.22;

    // found from sysid for one of the turn modules or tune by yourself
    // turn PID values for a swerve module
    public static final double kModuleTurn_P = 0.0081; 
    public static final double kModuleTurn_I = 0; 
    public static final double kModuleTurn_D = 0.00032; 

    // turn in place PID for the whole robot
    public static final double kTurn_P = 0.064; 
    public static final double kTurn_I = 0; 
    public static final double kTurn_D = 0.001; 
    public static final double kTurn_FF = 0; 
    public static final double kTurnErrorThreshold = 5.0; 
    public static final double kTurnVelocityThreshold = 0; 

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
    public static final double kDrive_D = 0.0001;

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

    public static final int kBeamBreakId = 3;

    public static final double kSuckerIntakeSpeed = 1;

    public static final double kShootSpeed = 5600; 
    public static final double kShootSuckerSpeed = 0.75; 
    public static final double kShootRevTime = 1; 
    public static final double kShootWaitTime = 0.5; 

    public static final double kAmpOuttakeSpeed = 0.3;
    public static final int kSuckerManualSpeed = 0;

    public static final double kP = 0.0003;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFF = 0.000215;
    public static final double kTolerance = 200; 
 
  }

  public class ArmConstants {
    public static final int kLimitSwitchId = 1;
    public static final int kArmMotorRightId = 13;
    public static final int kArmMotorLeftId = 14;

    public static final boolean kRightMotorInverted = false;
    public static final boolean kLeftMotorInverted = true; 

    public static final int kMotorCurrentLimit = 40;
    
    public static final float kForwardSoftLimit = 100;
    public static final double kReverseSoftLimit = 0; 

    public static final double kAbsPositionConversionFactor = 360;
    // velocity = position / 60
    public static final double kAbsVelocityConversionFactor = kAbsPositionConversionFactor / 60.0; 


    public static final double kRelativePositionConversionFactor = 360.0 / (5 * 5 * 4 * 5);
    // velocity = position / 60
    public static final double kRelativeVelocityConversionFactor = kRelativePositionConversionFactor / 60; 


    public static final double kTolerance = 2;

    public static final double kGroundPosition = 0;
    public static final double kTravelPosition = 20;
    public static final double kAmpPosition = 90;
    public static final double kSpeakerPosition = 10;

    public static final double kTravelSpeed = 0.5;


    // TODO: tune arm 
    public static final double kP = 2.5;
    public static final double kI = 0;
    public static final double kD = 0;

     
    // calculate using reca.lc
    // reduction: 500
    // CoM distance: 21.77 in
    // Arm mass: 18 lbs
    
    public static final double kS = 0; 
    public static final double kG = 0.014; // 0.79 V
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

    public static final float kFowardHangSoftLimit = 10;
    public static final float kReverseHangSoftLimit = 0;

    public static final double kHangSpeed = 0.25;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kHangMaxUp = 10;
    public static final double kHangInitial = 0;

    public static final double kHangTolerance = 2;
  }

  public static class Vision {
    public static final double kTranslationStdDevCoefficient = 0.35;
    public static final double kRotationStdDevCoefficient = 1;
    public static final AprilTagFieldLayout kAprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(); 
    public static final double limelightZHeight = 0; 
    public static final double kMaxAccuracyRange = 1000;

    public static final double confidence = 60;
    public static final Transform3d robotToCamera = new Transform3d();
    public static final OrderedPair kBlueAllianceLeftSpeakerCoordinate = new OrderedPair(0, 0);
    public static final OrderedPair kBlueAllianceRightSpeakerCoordinate = new OrderedPair(0, 0);
    public static final OrderedPair kRedAllianceLeftSpeakerCoordinate = new OrderedPair(0, 0);
    public static final OrderedPair kRedAllianceRightSpeakerCoordinate = new OrderedPair(0, 0);
    
    static {
      kAprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    }
  }

  public static class GeneralizedReleaseConstants {
    public static final Pose2d kRedAllianceSpeaker1 = new Pose2d(16.5, 4.97, new Rotation2d());
    public static final Pose2d kRedAllianceSpeaker2 = new Pose2d(16.5, 6.13, new Rotation2d());
    public static final Pose2d kBlueAllianceSpeaker1 = new Pose2d(0, 4.97, new Rotation2d());
    public static final Pose2d kBlueAllianceSpeaker2 = new Pose2d(0, 6.13, new Rotation2d());

    public static final Pose2d kRedAllianceSpeaker = new Pose2d(16.5, 5.54, new Rotation2d());
    public static final Pose2d kBlueAllianceSpeaker = new Pose2d(0, 5.54, new Rotation2d());

    public static final double blueShootRange = 5.87;
    public static final double redShootRange = 10.71;
    public static final double shootAnywhereTimeout = 7;
    public static final double waitAfterShot = 0.5;

    
    public static final InterpolatingDoubleTreeMap interpolationMap = new InterpolatingDoubleTreeMap(); 
    
    public static final Function<Double, Double> distanceToArmAngle = (dist) -> interpolationMap.get(dist); 
    // 5.82663 * Math.atan(3.94527 * dist - 7.66052) + 24.8349; 
    public static final BooleanSupplier readyToShoot = () -> Intake.getInstance().atDesiredShootSpeed() && Drivetrain.getInstance().atTargetAngle() && Arm.getInstance().isAtAngle(); 

    static {
      // interpolationMap.put(0, 0); 
      // interpolationMap.put(dist, angle);
      interpolationMap.put(1.37154,20.0);
      interpolationMap.put(2.0046,28.0);
      interpolationMap.put(2.37813, 34.0);
      interpolationMap.put(2.9402, 36.0);
      interpolationMap.put(3.5022, 39.0);
      interpolationMap.put(3.8638, 40.0);
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
    public static final int kLed2End = 18;
    public static final int kLed3Start = 18;
    public static final int kLed3End = 20;
    public static final int kLed4Start = 20;
    public static final int kLed4End = 40;

    public static final class Patterns {
      public static final LEDPattern kDefault = new SolidLEDPattern(LEDs.kDefaultColor);
      public static final LEDPattern kDisabled = new FadeLEDPattern(4, LEDs.kDefaultColor, kGold);
      public static final LEDPattern kAllianceRed = new SolidLEDPattern(Color.kRed);
      public static final LEDPattern kAllianceBlue = new SolidLEDPattern(Color.kBlue);
      public static final LEDPattern kEnabled = new SineLEDPattern(1, kGold, kCobaltBlue, 8);
      public static final LEDPattern kMoving = new FadeLEDPattern(1,kGold, Color.kWhite);
      public static final LEDPattern kIntake = new FadeLEDPattern(1,kCobaltBlue, Color.kGreen);
      public static final LEDPattern kMovingToNote = new RainbowPattern(1);
      public static final LEDPattern kShootAnywhere = new ConditionalPattern(new SolidLEDPattern(Color.kRed)).addCondition(Constants.GeneralizedReleaseConstants.readyToShoot::getAsBoolean, new SolidLEDPattern(Color.kGreen));
      public static final LEDPattern kArmMoving = new SolidLEDPattern(Color.kLightBlue);
      public static final LEDPattern kArmAtAmp = new SolidLEDPattern(Color.kSkyBlue);
      public static final LEDPattern kArmAtSpeaker = new SolidLEDPattern(Color.kBlue);
      public static final LEDPattern kArmAtGround = new SolidLEDPattern(Color.kDarkBlue);
      // public static final LEDPattern kShootAmp = new SolidLEDPattern(Color.kBlue);
      // public static final LEDPattern kShootSpeaker = new SolidLEDPattern(Color.kBlue);
      public static final LEDPattern kArmCustom = new SolidLEDPattern(Color.kSeaGreen);
      public static final LEDPattern kHangActive = new SineLEDPattern(2, Color.kPurple, Color.kBlack, 5);
      public static final LEDPattern kBeamHit = new StrobePattern(Color.kWhite, 0.5);
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
  public static final class AutoHang {
    public static final OrderedPair blueTopLeft1 = new OrderedPair(2.05, 5.93);
    public static final OrderedPair blueBottomRight1 = new OrderedPair(5.48, 5.55);
    public static final OrderedPair blueTopRight1 = new OrderedPair(4.56, 6.77);
    public static final OrderedPair blueBottomLeft1 = new OrderedPair(3.15, 4.50);

    public static final OrderedPair blueTopLeft2 = new OrderedPair(6.20, 5.50);
    public static final OrderedPair blueBottomRight2 = new OrderedPair(7.90, 3.33);
    public static final OrderedPair blueTopRight2 = new OrderedPair(7.90, 5.50);
    public static final OrderedPair blueBottomLeft2 = new OrderedPair(6.20, 3.33);


    public static final OrderedPair blueTopLeft3 = new OrderedPair(3.23, 3.92);
    public static final OrderedPair blueBottomRight3 = new OrderedPair(4.40, 1.56);
    public static final OrderedPair blueTopRight3 = new OrderedPair(5.32, 2.66);
    public static final OrderedPair blueBottomLeft3 = new OrderedPair(2.21, 2.79);

    public static final OrderedPair redTopRight1 = new OrderedPair(14.61, 5.93);
    public static final OrderedPair redBottomRight1 = new OrderedPair(13.30, 4.30);
    public static final OrderedPair redTopLeft1 = new OrderedPair(11.99, 6.77);
    public static final OrderedPair redBottomLeft1 = new OrderedPair(11.22, 5.55);

    public static final OrderedPair redTopLeft2 = new OrderedPair(9.20, 5.31);
    public static final OrderedPair redBottomRight2 = new OrderedPair(10.70, 2.80);
    public static final OrderedPair redTopRight2 = new OrderedPair(10.70, 5.31);
    public static final OrderedPair redBottomLeft2 = new OrderedPair(9.20, 2.80);


    public static final OrderedPair redTopLeft3 = new OrderedPair(11.21, 2.55);
    public static final OrderedPair redBottomRight3 = new OrderedPair(14.26, 2.55);
    public static final OrderedPair redTopRight3 = new OrderedPair(13.35, 3.66);
    public static final OrderedPair redBottomLeft3 = new OrderedPair(12.32, 1.22);
  }
  
  public static class Source {
    public static final OrderedPair redSourceTL = new OrderedPair(0, 2.2); 
    public static final OrderedPair redSourceBR = new OrderedPair(3.27, 0); 

    public static final OrderedPair blueSourceTL = new OrderedPair(13.23, 2.2); 
    public static final OrderedPair blueSourceBR = new OrderedPair(16.5, 0); 
  }
  
}
