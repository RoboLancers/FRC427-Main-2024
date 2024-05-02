// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.Function;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.SwerveModuleConfig;
import frc.robot.subsystems.intake.Intake;
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
      new Translation2d(kWheelBaseMeters / 2, kTrackWidthMeters / 2), // front left
      new Translation2d(kWheelBaseMeters / 2, -kTrackWidthMeters / 2), // front right
      new Translation2d(-kWheelBaseMeters / 2, kTrackWidthMeters / 2), // back left
      new Translation2d(-kWheelBaseMeters / 2, -kTrackWidthMeters / 2) // back right
    ); 

    // copied speeds (https://github.com/SwerveDriveSpecialties/swerve-template-2021-unmaintained/blob/master/src/main/java/frc/robot/subsystems/DrivetrainSubsystem.java)
    public static final double kMaxAttainableModuleSpeedMetersPerSecond = 5880.0 / 60.0 / kGearRatio *
    2 * Units.inchesToMeters(kWheelRadiusInches) * Math.PI; // max attainable speed for each drive motor
    public static final double kMaxAttainableRotationRadPerSecond = kMaxAttainableModuleSpeedMetersPerSecond /
    Math.hypot(kTrackWidthMeters / 2.0, kWheelBaseMeters / 2.0); // max rotation of robot
    
    public static double kMaxSpeedMetersPerSecond = 0.85; // max velocity (no turning) of robot; may tune to be a fraction of the attainable module speed
    public static double kMaxSlowSpeedMetersPerSecond = 1.0; 
    public static final double kMaxAccelerationMetersPerSecondSquared = kMaxSpeedMetersPerSecond / 0.05; // max acceleration of robot (accelerate to max speed in 1 second)
    public static double kMaxRotationRadPerSecond = 2.50; // 3.00; // max rotation speed of the robot
    public static final double kMaxSlowRotationRadPerSecond = Math.PI / 2; 
    public static final double kMaxRotationAccelerationRadPerSecondSquared = kMaxRotationRadPerSecond / 0.001; // max angular acceleration of robot

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
    public static final double kTurn_P = 0.09; 
    public static final double kTurn_I = 0; 
    public static final double kTurn_D = 0.0015; 
    public static final double kTurn_FF = 0; 
    public static final double kTurnErrorThreshold = 5.0; 
    public static final double kTurnVelocityThreshold = 0; 

    // current limits for each motor
    public static final int kDriveCurrentLimit = 40; 
    public static final int kDriveSecondaryLimit = 50; 
    public static final double kDriveRampRate = 0.05; 
     
    public static final int kTurnCurrentLimit = 20; 
    public static final double kTurnRampRate = 0.05;

    // max acceleration/deceleration of each motor (used for high CG robots)
    public static final double kForwardSlewRate = kMaxAccelerationMetersPerSecondSquared; 
    public static final double kStrafeSlewRate = kMaxAccelerationMetersPerSecondSquared; 
    public static final double kTurnSlewRate = kMaxRotationAccelerationRadPerSecondSquared; 
  }

  public static class Trajectory {
    // translational PID of robot for trajectory use
    public static final double kDrive_P = 5.25; 
    public static final double kDrive_I = 0; 
    public static final double kDrive_D = 0.0005;

    // angular PID (same as turn pid)
    public static final double kOmega_P = 3.25; 
    public static final double kOmega_I = 0; 
    public static final double kOmega_D = 0.0001; 

    // max velocity & acceleration robot can go n following a trajectory
    public static final double kMaxVelocityMetersPerSecond = 3; 
    public static final double kMaxAccelerationMetersPerSecondSquared = 2.5; 

    public static final double kMaxAngularVelocityRadiansPerSecond = Units.degreesToRadians(360); 
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Units.degreesToRadians(360); 
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
    public static final int kSuckOuttakeMotorLimit = 60;

    public static final double kShootVelocityConversionFactor = 1; 
    public static final double kIntakeVelocityConversionFactor = 1; 

    public static final int kBeamBreakId = 3;

    public static final double kSuckerIntakeSpeed = 0.75;

    public static final double kShootSpeed = 5600; 
    public static final double kShootSuckerSpeed = 0.4; 
    public static final double kShootRevTime = 1; 
    public static final double kShootWaitTime = 0.5; 

    public static final double kAmpOuttakeSpeed = 0.25;
    public static final int kSuckerManualSpeed = 0;

    public static final double kTopP = 0.00004; // 0.000030;
    public static final double kTopI = 0;
    public static final double kTopD = 0;
    public static final double kTopFF = 0.0001750357; // 0.0002025 * 31.25 / 35.0;
    public static final double kTolerance = 100; 
    
    public static final double kBottomP = 0.00004; // 0.000030;
    public static final double kBottomI = 0;
    public static final double kBottomD = 0;
    public static final double kBottomFF = 0.000180; // 0.0002025 * 31.25 / 35.0;
  }

  public class ArmConstants {
    public static final int kLimitSwitchId = 1;
    public static final int kArmMotorRightId = 14;
    public static final int kArmMotorLeftId = 13;

    public static final boolean kRightMotorInverted = true;
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


    public static final double kTolerance = 0.5;

    public static final double kGroundPosition = 0;
    public static final double kTravelPosition = 20;
    public static final double kAmpPosition = 90;
    public static final double kSpeakerPosition = 19;

    public static final double kTravelSpeed = 0.5;


    public static final double kP = 0.03;
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
    public static final int kHangMotorID = 18;

    public static final boolean kMotorInverted = false;

    public static final int kHangMotorLimit = 40;

    public static final double kPositionConversionFactor = 1;
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60;

    public static final float kFowardHangSoftLimit = 0;
    public static final float kReverseHangSoftLimit = 0;

    public static final double kHangSpeed = 0.25;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kHangMaxUp = 10;
    public static final double kHangInitial = 0;

    public static final double kHangTolerance = 2;

    public static final double kHookStallCurrent = 7.5; 
    public static final double kMinSpeed = 1; 
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

    public static final double kFrontPitchDegrees = -10; 
    public static final double kCameraHeightMeters = 0.25; 
    
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

    public static final Pose2d kRedAllianceSpeakerTarget = new Pose2d(16.5 - 0.15, 5.54, new Rotation2d()); 
    public static final Pose2d kBlueAllianceSpeakerTarget = new Pose2d(0.15, 5.54, new Rotation2d()); 

    public static final double blueShootRange = 5.87;
    public static final double redShootRange = 10.71;
    public static final double shootAnywhereTimeout = 7;
    public static final double waitAfterShot = 0.5;

    
    public static final InterpolatingDoubleTreeMap armInterpolationMap = new InterpolatingDoubleTreeMap(); 

    public static final InterpolatingDoubleTreeMap flywheelInterpolationMap = new InterpolatingDoubleTreeMap(); 
    
    public static final Function<Double, Double> distanceToArmAngle = (dist) -> armInterpolationMap.get(dist); 

    public static final Function<Double, Double> distanceToFlywheelSpeed = (dist) -> flywheelInterpolationMap.get(dist); 

    // 5.82663 * Math.atan(3.94527 * dist - 7.66052) + 24.8349; 
    public static final BooleanSupplier readyToShoot = () -> Intake.getInstance().atDesiredShootSpeed() && Drivetrain.getInstance().atTargetAngle() && Arm.getInstance().isAtAngle(); 

    public static final BooleanSupplier readyToShootAuto = () -> Intake.getInstance().atDesiredShootSpeed() && Arm.getInstance().isAtAngle(); 

    static {
      // armInterpolationMap.put(0, 0); 
      // armInterpolationMap.put(dist, angle);
      // armInterpolationMap.put(0.91, 17.0);
      // armInterpolationMap.put(1.37154,19.0);
      // armInterpolationMap.put(2.0046,27.0);
      // armInterpolationMap.put(2.37813, 30.5);
      // armInterpolationMap.put(2.5268, 32.2);
      // armInterpolationMap.put(2.9402, 33.5);
      // armInterpolationMap.put(3.5022, 38.0);
      // armInterpolationMap.put(3.8638, 39.0);

      // flywheelInterpolationMap.put(1.1769, 2600.0); // 3100
      // flywheelInterpolationMap.put(1.4206, 2700.0); // 3300
      // flywheelInterpolationMap.put(1.6509, 2750.0); // 3400
      // flywheelInterpolationMap.put(1.7909, 2775.0); // 3500
      // flywheelInterpolationMap.put(1.9566, 2790.0); // 3600
      // flywheelInterpolationMap.put(2.2225, 2800.0); // 3650
      // flywheelInterpolationMap.put(2.5153, 3200.0); // 3700
      // flywheelInterpolationMap.put(2.7, 5600.0);

      //OLD
      // 1.1769, 2600
      // 1.4206, 2700
      // 1.6509,2750
      // 1.7909,2775
      // 1.9566,2790
      // 2.2225,2800
      // 2.5153, 3200
      //
      //NEW
      //1.0519,2400
      //1.219,2400
      //1.432,2400
      //1.695,2500
      //1.93,2500
      //2.18,2600
      //2.31,2600
      //2.5268,2900

      // double kArmAdjust = 0; // -1.25; 

      // armInterpolationMap.put(1.160310, 17.0 + kArmAdjust);
      // armInterpolationMap.put(1.324259, 18.0 + kArmAdjust);
      // armInterpolationMap.put(1.528990, 20.5 + kArmAdjust);
      // armInterpolationMap.put(1.764248, 24.5 + kArmAdjust);
      // armInterpolationMap.put(1.961984, 26.5 + kArmAdjust);
      // armInterpolationMap.put(2.242998, 30.0 + kArmAdjust);
      // armInterpolationMap.put(2.545966, 34.0 + kArmAdjust);
      // armInterpolationMap.put(2.8188528, 34.9 + kArmAdjust);
      // armInterpolationMap.put(3.189439307, 37.95 + kArmAdjust);
      // armInterpolationMap.put(3.461469, 38.27175 + kArmAdjust);
      // armInterpolationMap.put(3.698021, 39.13 + kArmAdjust);

      // double kFlywheelAdjust = 0; // -50; 

      // flywheelInterpolationMap.put(1.160310, 2500.0 + kFlywheelAdjust);
      // flywheelInterpolationMap.put(1.324259, 2500.0 + kFlywheelAdjust);
      // flywheelInterpolationMap.put(1.528990, 2600.0 + kFlywheelAdjust);
      // flywheelInterpolationMap.put(1.764248, 2600.0 + kFlywheelAdjust);
      // flywheelInterpolationMap.put(1.961984, 2700.0 + kFlywheelAdjust);
      // flywheelInterpolationMap.put(2.242998, 2800.0 + kFlywheelAdjust);
      // flywheelInterpolationMap.put(2.545966, 2900.0 + kFlywheelAdjust);
      // flywheelInterpolationMap.put(2.8188528, 3100.0 + kFlywheelAdjust);
      // flywheelInterpolationMap.put(3.189439307, 3380.0 + kFlywheelAdjust);
      // flywheelInterpolationMap.put(3.461469, 3600.0 + kFlywheelAdjust);
      // flywheelInterpolationMap.put(3.698021, 4000.0 + kFlywheelAdjust);

      double kArmOffset = -4.0; 

      // armInterpolationMap.put(1.0, 16.5 + kArmOffset);
      // armInterpolationMap.put(1.261146, 17.0 + kArmOffset);
      // armInterpolationMap.put(1.555493, 22.5 + kArmOffset);
      // armInterpolationMap.put(1.749017, 24.0 + kArmOffset);
      // armInterpolationMap.put(1.945373, 29.0 + kArmOffset);
      // armInterpolationMap.put(2.268227, 33.0 + kArmOffset);
      // armInterpolationMap.put(2.533997, 36.0 + kArmOffset);
      // armInterpolationMap.put(2.85052, 37.5 + kArmOffset);
      // armInterpolationMap.put(2.98085, 39.0 + kArmOffset);
      // armInterpolationMap.put(3.222661, 41.3 + kArmOffset);

      // flywheelInterpolationMap.put(1.0, 2400.0);
      // flywheelInterpolationMap.put(1.261146, 2400.0);
      // flywheelInterpolationMap.put(1.555493, 2400.0);
      // flywheelInterpolationMap.put(1.749017, 2500.0);
      // flywheelInterpolationMap.put(1.945373, 2600.0);
      // flywheelInterpolationMap.put(2.268227, 2800.0);
      // flywheelInterpolationMap.put(2.533997, 2800.0);
      // flywheelInterpolationMap.put(2.85052, 2800.00);
      // flywheelInterpolationMap.put(2.98085, 3200.0);
      // flywheelInterpolationMap.put(3.222661, 3200.0);

      // armInterpolationMap.put(1.337, 21.5);
      // armInterpolationMap.put(1.714, 30.0);
      // armInterpolationMap.put(2.16038, 35.0);
      // armInterpolationMap.put(2.631287, 38.5);
      // armInterpolationMap.put(3.00, 40.0);

      // flywheelInterpolationMap.put(1.337, 2400.0);
      // flywheelInterpolationMap.put(1.714, 2500.0);
      // flywheelInterpolationMap.put(2.16038, 2500.0);
      // flywheelInterpolationMap.put(2.631287, 2600.0);
      // flywheelInterpolationMap.put(3.00, 2800.0);

      // armInterpolationMap.put(1.236991, 19.0);
      // armInterpolationMap.put(1.475904, 22.0);
      // armInterpolationMap.put(1.731548, 27.0);
      // armInterpolationMap.put(1.945692, 30.0);
      // armInterpolationMap.put(2.160266, 32.0);
      // armInterpolationMap.put(2.346438, 34.75);
      // armInterpolationMap.put(2.551097, 36.75);
      // armInterpolationMap.put(2.780076, 38.0);
      // armInterpolationMap.put(3.013878, 40.0);
      // armInterpolationMap.put(3.179525, 41.75);
      // armInterpolationMap.put(3.395265, 42.65);
      // armInterpolationMap.put(3.597070, 43.85);

      // flywheelInterpolationMap.put(1.236991, 2400.0);
      // flywheelInterpolationMap.put(1.475904, 2400.0);
      // flywheelInterpolationMap.put(1.731548, 2400.0);
      // flywheelInterpolationMap.put(1.945692, 2500.0);
      // flywheelInterpolationMap.put(2.160266, 2500.0);
      // flywheelInterpolationMap.put(2.346438, 2550.0);
      // flywheelInterpolationMap.put(2.551097, 2550.0);
      // flywheelInterpolationMap.put(2.780076, 2650.0);
      // flywheelInterpolationMap.put(3.013878, 2650.0);
      // flywheelInterpolationMap.put(3.179525, 2775.0);
      // flywheelInterpolationMap.put(3.395265, 3000.0);
      // flywheelInterpolationMap.put(3.597070, 3200.0);

      armInterpolationMap.put(1.217, 17.0); 
      armInterpolationMap.put(1.589, 19.0); 
      armInterpolationMap.put(2.079, 28.0); 
      armInterpolationMap.put(2.450, 37.0); 
      armInterpolationMap.put(2.782, 41.0);
      armInterpolationMap.put(3.329, 43.5);

      flywheelInterpolationMap.put(1.217, 2400.0);
      flywheelInterpolationMap.put(1.589, 2400.0);
      flywheelInterpolationMap.put(2.079, 2400.0);
      flywheelInterpolationMap.put(2.450, 2900.0);
      flywheelInterpolationMap.put(2.782, 2900.0);
      flywheelInterpolationMap.put(3.329, 3100.0);
      
      
      





      // 1.337, 21.5, 2400
      // 1.714, 30, 2500
      // 2.16038, 35, 2500
      // 2.631287, 38.5, 2600
      // 3.00, 40, 2800
    }
  }

  public static final class LEDs {
    public static final Color kDefaultColor = Color.kDarkBlue;
    public static final Color kCobaltBlue = new Color("#0047AB");
    public static final Color kGold = new Color(142, 66, 0); // 142.17, 66.44

    public static final int kLedPort = 6; 
    public static final int kLedLength = 30; 

    public static final int kLed1Start = 0; 
    public static final int kLed1End = 3; 
    public static final int kLed2Start = 3; 
    public static final int kLed2End = 4;
    public static final int kLed3Start = 0;
    public static final int kLed3End = 11;
    public static final int kLed4Start = 11;
    public static final int kLed4End = 25;

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
      public static final LEDPattern kArmMoving = new SolidLEDPattern(Color.kGold); //3
      public static final LEDPattern kArmAtAmp = new SolidLEDPattern(kDefaultColor); //4
      public static final LEDPattern kArmAtSpeaker = new SolidLEDPattern(Color.kYellow); //2
      public static final LEDPattern kArmAtGround = new SolidLEDPattern(Color.kLightYellow); //Lightest to Darkest (1-4) 1
      // public static final LEDPattern kShootAmp = new SolidLEDPattern(Color.kBlue);
      // public static final LEDPattern kShootSpeaker = new SolidLEDPattern(Color.kBlue);
      public static final LEDPattern kArmCustom = new SolidLEDPattern(Color.kSeaGreen);
      public static final LEDPattern kHangActive = new SineLEDPattern(2, Color.kPurple, Color.kBlack, 5);
      public static final LEDPattern kBeamHit = new StrobePattern(Color.kWhite, 0.125);
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
    public static final Pose2d ampBlue = new Pose2d(1.83, 7.4, Rotation2d.fromDegrees(-90));
    public static final Pose2d ampRed = new Pose2d(14.7, 7.4, Rotation2d.fromDegrees(-90));
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
  
  public static class SetPoints {
    public static final Pose2d blueThirdMiddle = new Pose2d(2.55, 3.15,null);
    public static final Pose2d blueSecondMiddle = new Pose2d(3.59, 5.80, null);
    public static final Pose2d blueFirstMiddle = new Pose2d(3.38, 7.02, null);
    public static final Pose2d blueCenter = new Pose2d(1.31, 5.50, null);

  }
}
