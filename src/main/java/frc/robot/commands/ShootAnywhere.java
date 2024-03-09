package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.commands.GoToAngle;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.TurnToAngle;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.OuttakeToSpeaker;
import frc.robot.util.GeometryUtils;
import frc.robot.util.quad.OrderedPair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShootAnywhere { 

    public static Command shootAnywhere(Drivetrain drivetrain, Arm arm, Intake intake) {
        return shootAnywhere(drivetrain, arm, intake, 1); 
    }

    public static Command shootAnywhere(Drivetrain drivetrain, Arm arm, Intake intake, double speed) {
        ShootAnywhereResult res = getShootValues(drivetrain.getPose()); 

        if (res == null) return Commands.none();

        TurnToAngle turnToAngle = new TurnToAngle(drivetrain, res.getDriveAngleDeg());
        GoToAngle goToAngle = new GoToAngle(arm, res.getArmAngleDeg());
        // Command outtake = OuttakeToSpeaker.outtakeToSpeaker(intake);
        Command rev = OuttakeToSpeaker.revAndIndex(intake).withTimeout(1.5); 
        Command shoot = OuttakeToSpeaker.shoot(intake, 0.5); 

        return Commands.sequence(drivetrain.zeroDrivetrain(), Commands.parallel(turnToAngle, goToAngle, rev), drivetrain.zeroDrivetrain(), shoot)
        .finallyDo(() -> {
            arm.goToAngle(Constants.ArmConstants.kTravelPosition);
        });
    }

    public static ShootAnywhereResult getShootValues(Pose2d currentPose) {
        Pose2d targetPose = null;
        Pose2d targetArmPose = null; 

        Optional<Alliance> optAlliance = DriverStation.getAlliance(); 

        if (optAlliance.isEmpty()) return null;

        Alliance alliance = optAlliance.get();

        /*
         * Option 1: 
         *  - target drive & arm with bisector
         * Option 2: 
         *  - target drive & arm speaker w/o offset
         * Option 3: 
         *  - target drive speaker w/ offset
         *  - target arm speaker w/o offset
         * Option 4: 
         *  - target drive & arm speaker w/ offset
         */

        if (alliance == DriverStation.Alliance.Blue) {
            // targetPose = Constants.GeneralizedReleaseConstants.kBlueAllianceSpeaker;
            // targetPose = Constants.GeneralizedReleaseConstants.kBlueAllianceSpeakerTarget;
            targetPose = GeometryUtils.getBisector(
            OrderedPair.fromPose2d(currentPose), 
            OrderedPair.fromPose2d(Constants.GeneralizedReleaseConstants.kBlueAllianceSpeaker1), 
            OrderedPair.fromPose2d(Constants.GeneralizedReleaseConstants.kBlueAllianceSpeaker2)).toPose2d(); 
            targetArmPose = targetPose; 
            // targetArmPose = Constants.GeneralizedReleaseConstants.kRedAllianceSpeaker;
        } else if (alliance == DriverStation.Alliance.Red) {
            // targetPose = Constants.GeneralizedReleaseConstants.kRedAllianceSpeaker;
            // targetPose = Constants.GeneralizedReleaseConstants.kRedAllianceSpeakerTarget;
            targetPose = GeometryUtils.getBisector(
            OrderedPair.fromPose2d(currentPose), 
            OrderedPair.fromPose2d(Constants.GeneralizedReleaseConstants.kRedAllianceSpeaker1), 
            OrderedPair.fromPose2d(Constants.GeneralizedReleaseConstants.kRedAllianceSpeaker2)).toPose2d(); 
            targetArmPose = targetPose;
            // targetArmPose = Constants.GeneralizedReleaseConstants.kRedAllianceSpeaker;
        }
        if (targetPose == null) return null;

        double finalAngle = Math.atan2(currentPose.getY() - targetPose.getY(),  currentPose.getX() - targetPose.getX());
        double distance = Math.hypot(currentPose.getY() - targetArmPose.getY(), currentPose.getX() - targetArmPose.getX()); 

        double angleToTurnArm = Constants.GeneralizedReleaseConstants.distanceToArmAngle.apply(distance);

        double speedToRevFlywheel = Constants.GeneralizedReleaseConstants.distanceToFlywheelSpeed.apply(distance); 

        SmartDashboard.putNumber("Shoot Anywhere Arm Angle", angleToTurnArm); 
        SmartDashboard.putNumber("Shoot Anywhere Distance", distance); 

        return new ShootAnywhereResult(Math.toDegrees(finalAngle), angleToTurnArm, speedToRevFlywheel); 
    }

    public static class ShootAnywhereResult {
        private double drivetrainAngle; 
        private double armAngle; 
        private double outtakeSpeed;

        public ShootAnywhereResult(double drivetrainAngleDeg, double armAngleDeg, double outtakeSpeed) {
            this.drivetrainAngle = drivetrainAngleDeg; 
            this.armAngle = armAngleDeg; 
            this.outtakeSpeed = outtakeSpeed;
        }

        public double getOuttakeSpeed() {
            return this.outtakeSpeed;
        }

        public double getDriveAngleDeg() {
            return this.drivetrainAngle; 
        }

        public double getArmAngleDeg() {
            return this.armAngle; 
        }
    }

}
