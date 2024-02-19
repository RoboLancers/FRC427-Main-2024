package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.SetShooterSpeed;
import frc.robot.subsystems.intake.commands.SetSuckerIntakeSpeed;
import frc.robot.util.ChassisState;
import frc.robot.util.DriverController;

public class UpdatedShootAnywhere extends Command {
    
    public Drivetrain drivetrain;
    public Arm arm;
    public Intake intake;
    public DriverController driverController;
    public Optional<Alliance> optAlliance;
    public Timer timer = new Timer(); 
    public Pose2d targetPose; 

    public UpdatedShootAnywhere(DriverController driverController, Drivetrain drivetrain, Arm arm, Intake intake) {
        this.arm = arm;
        this.intake = intake;
        this.drivetrain = drivetrain;
        this.driverController = driverController;

        addRequirements(drivetrain, arm, intake);
    }

    public void initialize() {
        timer.reset();
        timer.start();
        intake.outtakeRing(Constants.IntakeConstants.kShootSuckerSpeed);
        this.optAlliance = DriverStation.getAlliance();
        Alliance alliance = optAlliance.get();
        if (alliance == DriverStation.Alliance.Blue) {
            targetPose = Constants.Vision.kBlueAllianceSpeaker;
        }
        else if (alliance == DriverStation.Alliance.Red) {
            targetPose = Constants.Vision.kRedAllianceSpeaker;
        }
    }

    public void execute() {
        Pose2d currentPose = drivetrain.getPose();
        double finalAngle = Math.atan2(currentPose.getY() - targetPose.getY(),  currentPose.getX() - targetPose.getX());
        double distance = Math.hypot(currentPose.getY() - targetPose.getY(), currentPose.getX() - targetPose.getX());
        double angleToTurnArm = Constants.Vision.distanceToArmAngle.apply(distance);
        arm.goToAngle(angleToTurnArm);
        ChassisState speeds = driverController.getDesiredChassisState(); 
        speeds.omegaRadians = finalAngle;
        speeds.turn = true; 
        drivetrain.swerveDriveFieldRel(speeds, true);
    }

    public boolean isFinished() {
        return timer.get() > Constants.Vision.shootAnywhereTimeout || optAlliance.isEmpty() || targetPose == null;
    }

    public void end(boolean interrupted) {
        timer.stop();
        boolean isInRange = false;
        if (this.optAlliance.get() == DriverStation.Alliance.Blue) {
            isInRange = drivetrain.getPose().getX() <= Constants.Vision.blueShootRange;
        }
        else if (this.optAlliance.get() == DriverStation.Alliance.Red) {
            isInRange = drivetrain.getPose().getX() >= Constants.Vision.redShootRange;
        }

        if (!isInRange || !interrupted) {
            intake.stopShoot();
            arm.goToAngle(Constants.ArmConstants.kTravelPosition);
            CommandScheduler.getInstance().schedule(DriverCommands.vibrateController(driverController.getHID(), 1));
        } else {
            SetSuckerIntakeSpeed suckerSpeed = new SetSuckerIntakeSpeed(intake, 1);
            Command command = Commands.sequence(
                suckerSpeed, 
                new WaitCommand(Constants.Vision.waitAfterShot), 
                new SetSuckerIntakeSpeed(intake, 0), 
                new SetShooterSpeed(intake, 0)
            ).finallyDo(() -> {
                arm.goToAngle(Constants.ArmConstants.kTravelPosition);
            }); 
            CommandScheduler.getInstance().schedule(command);
        }
    }
}
