package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hang.Hang;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.OuttakeToSpeaker;
import frc.robot.subsystems.leds.Led;
import frc.robot.util.ChassisState;
import frc.robot.util.DriverController;
import frc.robot.util.GeometryUtils;

public class GeneralizedTrapRoutine extends Command {
    
    public DriverController driverController;
    public Drivetrain drivetrain;
    public Arm arm;
    private Intake intake; 
    public Hang hang; 
    private double angleToTurn; 

    public GeneralizedTrapRoutine(DriverController driverController, Drivetrain drivetrain, Arm arm, Intake intake) {
        this.arm = arm; 
        this.driverController = driverController;
        this.drivetrain = drivetrain;
        this.intake = intake; 

        addRequirements(drivetrain, hang); 
    }

    public void initialize() {
        this.angleToTurn = GeometryUtils.getAngleToStage(drivetrain.getPose()); 
        this.arm.goToAngle(Constants.ArmConstants.kTrapPosition);
        this.intake.outtakeRing(Constants.IntakeConstants.kTrapSpeed);
        Led.getInstance().isShooting = true; 
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
        Led.getInstance().isShooting = false; 
        // this.hang.setTargetPosition(Constants.HangConstants.kHangInitial);
        Command command = OuttakeToSpeaker.shoot(intake).finallyDo(() -> {
                intake.stopShoot();
                intake.stopSuck();
                arm.goToAngle(Constants.ArmConstants.kTravelPosition);
            }); 
        CommandScheduler.getInstance().schedule(command);
    }
}
