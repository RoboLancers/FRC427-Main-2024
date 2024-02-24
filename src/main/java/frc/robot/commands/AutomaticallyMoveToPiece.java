package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.vision.FrontVision;
import frc.robot.util.ChassisState;
import frc.robot.util.DriverController;

public class AutomaticallyMoveToPiece {

    public static Command automaticallyMoveToPiece(DriverController driverController, Drivetrain drivetrain) {
        // var result = frontVision.getLatestVisionResult();
        // if (!result.hasTargets()) return Commands.none();
        
        double angleToTurn = 30;
        double actualAngle = angleToTurn + drivetrain.getPose().getRotation().getDegrees();

        System.out.println(actualAngle);


        return new ParallelCommandGroup(Commands.run(() -> {

            ChassisState speeds = driverController.getDesiredChassisState();

            ChassisState finalState = new ChassisState(
                speeds.vxMetersPerSecond * Math.cos(Math.toRadians(actualAngle)) - speeds.vyMetersPerSecond * Math.sin(Math.toRadians(actualAngle)), 
                speeds.vxMetersPerSecond * Math.sin(Math.toRadians(actualAngle)) + speeds.vyMetersPerSecond * Math.cos(Math.toRadians(actualAngle)), Math.toRadians(actualAngle), true);
            drivetrain.swerveDriveFieldRel(finalState, false);
        }, drivetrain), AutomationCommands.autoIntakeCommand()); // Any processing before turning to that angle
    }
}
