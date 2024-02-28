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

    public static Command waitForVision(FrontVision frontVision) {
        return Commands.waitUntil(() -> {
            var result = frontVision.getLatestVisionResult();
            return result.hasTargets(); 
        }); 
    }

    public static Command automaticallyMoveToPiece(DriverController driverController, Drivetrain drivetrain, FrontVision frontVision) {
        var result = frontVision.getLatestVisionResult();
        if (!result.hasTargets()) return Commands.none();
        
        double angleToTurn = - frontVision.getNoteRotation();
        double actualAngle = angleToTurn + drivetrain.getPose().getRotation().getDegrees();

        System.out.println(actualAngle);


        return new ParallelCommandGroup(Commands.run(() -> {

            ChassisState driverInput = driverController.getDesiredChassisState();
            driverInput.vyMetersPerSecond = 0; 

            drivetrain.swerveDriveFieldRel(new ChassisState(
                driverInput.vxMetersPerSecond * Math.cos(Math.toRadians(actualAngle)) - driverInput.vyMetersPerSecond * Math.sin(Math.toRadians(actualAngle)), 
                driverInput.vxMetersPerSecond * Math.sin(Math.toRadians(actualAngle)) + driverInput.vyMetersPerSecond * Math.cos(Math.toRadians(actualAngle)), 
                Math.toRadians(actualAngle), true
                ), false, false);
        }, drivetrain), AutomationCommands.autoIntakeCommand()); // Any processing before turning to that angle
    }
}
