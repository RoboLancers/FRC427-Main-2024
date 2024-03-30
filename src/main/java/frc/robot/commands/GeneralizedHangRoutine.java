package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hang.Hang;
import frc.robot.subsystems.hang.commands.SetHangSpeed;
import frc.robot.util.ChassisState;
import frc.robot.util.DriverController;
import frc.robot.util.GeometryUtils;
import frc.robot.util.quad.OrderedPair;
import frc.robot.util.quad.Quadrilateral;

public class GeneralizedHangRoutine extends Command {
    
    public DriverController driverController;
    public Drivetrain drivetrain;
    public Arm arm;
    public Hang hang; 
    private double angleToTurn; 

    public GeneralizedHangRoutine(DriverController driverController, Drivetrain drivetrain, Arm arm, Hang hang) {
        this.arm = arm; 
        this.hang = hang;
        this.driverController = driverController;
        this.drivetrain = drivetrain;

        addRequirements(drivetrain, hang); 
    }

    public void initialize() {
        this.angleToTurn = GeometryUtils.getAngleToStage(drivetrain.getPose()); 

        // this.hang.setTargetPosition(Constants.HangConstants.kHangMaxUp);
    }

    public void execute() {
        ChassisState speeds = driverController.getDesiredChassisState(); 
        speeds.omegaRadians = Math.toRadians(this.angleToTurn);
        speeds.turn = true;
        speeds.vxMetersPerSecond *= -1; 
        speeds.vyMetersPerSecond *= -1; 
        ChassisState finalState = new ChassisState(
            speeds.vxMetersPerSecond * Math.cos(Math.toRadians(this.angleToTurn)) - speeds.vyMetersPerSecond * Math.sin(Math.toRadians(this.angleToTurn)), 
            speeds.vxMetersPerSecond * Math.sin(Math.toRadians(this.angleToTurn)) + speeds.vyMetersPerSecond * Math.cos(Math.toRadians(this.angleToTurn)), speeds.omegaRadians, true);
        drivetrain.swerveDriveFieldRel(finalState, false, false);
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted) {
        // this.hang.setTargetPosition(Constants.HangConstants.kHangInitial);
    }

    public static Command primeHang(Hang hang) {
        return new SetHangSpeed(hang, 0.5).alongWith(Commands.waitUntil(() -> 
            hang.getMotorCurrent() >= Constants.HangConstants.kHookStallCurrent && Math.abs(hang.getHangVelocity()) <= Constants.HangConstants.kMinSpeed
        ))
        .andThen(new SetHangSpeed(hang, 0)); 
    }
}
