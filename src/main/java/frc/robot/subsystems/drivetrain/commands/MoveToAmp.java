package frc.robot.subsystems.drivetrain.commands;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;

public class MoveToAmp {
    public static Pose2d getTargetPose() {
        Optional<Alliance> optAlliance = DriverStation.getAlliance();

        if (optAlliance.isEmpty()) return null;

        Alliance alliance = optAlliance.get(); 

        Pose2d targetPose = null;

        if (alliance == Alliance.Blue) {
            targetPose = Constants.PathFollower.ampBlue;
        }
        if (alliance == Alliance.Red) {
            targetPose = Constants.PathFollower.ampRed;
        }

        return targetPose;

    }

    public static Command goToAmp() {
        Pose2d targetPose = getTargetPose();

        if (targetPose == null) {
            return Commands.none(); 
        }

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
            Constants.Trajectory.kMaxVelocityMetersPerSecond, 
            Constants.Trajectory.kMaxAccelerationMetersPerSecondSquared,
            Constants.Trajectory.kMaxAngularVelocityRadiansPerSecond, 
            Constants.Trajectory.kMaxAngularAccelerationRadiansPerSecondSquared
        );

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0, // Goal end velocity in meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );
        return pathfindingCommand;
    }
}


