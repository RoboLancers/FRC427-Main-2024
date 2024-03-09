package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.commands.ShootAnywhere.ShootAnywhereResult;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.OuttakeToSpeaker;
import frc.robot.subsystems.intake.commands.SetShooterSpeed;
import frc.robot.util.ChassisState;
import frc.robot.util.DriverController;
import frc.robot.util.GeometryUtils;
import frc.robot.util.quad.OrderedPair;

public class GeneralizedReleaseRoutine extends Command {
    
    public Drivetrain drivetrain;
    public Arm arm;
    public Intake intake;
    public DriverController driverController;
    public Optional<Alliance> optAlliance;
    public Timer timer = new Timer();

    public GeneralizedReleaseRoutine(DriverController driverController, Drivetrain drivetrain, Arm arm, Intake intake) {
        this.arm = arm;
        this.intake = intake;
        this.drivetrain = drivetrain;
        this.driverController = driverController;

        addRequirements(drivetrain, arm);
    }

    // gets alliance color
    // revs shooter wheels
    public void initialize() {
        timer.reset();
        timer.start();
        CommandScheduler.getInstance().schedule(SetShooterSpeed.revAndIndex(intake, Constants.IntakeConstants.kShootSpeed));
        this.optAlliance = DriverStation.getAlliance();
    }

    // gets the driver pose
    // rotates the robot to the angle to face the speaker & turns the arm to the angle 
    // sets the driver speed to robot controller input
    public void execute() {
        Pose2d currentPose = drivetrain.getPose();
        ShootAnywhereResult results = ShootAnywhere.getShootValues(currentPose);

        boolean restrictedFromShooting = GeometryUtils.isInBlueStage(OrderedPair.fromPose2d(currentPose)) || GeometryUtils.isInRedStage(OrderedPair.fromPose2d(currentPose)) || (drivetrain.getPose().getX() >= Constants.GeneralizedReleaseConstants.blueShootRange && drivetrain.getPose().getX() <= Constants.GeneralizedReleaseConstants.redShootRange); 

        if (!restrictedFromShooting) {
            arm.goToAngle(results.getArmAngleDeg());
        }
        else {
            arm.goToAngle(Constants.ArmConstants.kTravelPosition);
        }
        ChassisState speeds = driverController.getDesiredChassisState(); 
        speeds.omegaRadians = Math.toRadians(results.getDriveAngleDeg());
        speeds.turn = true;
        drivetrain.swerveDriveFieldRel(speeds, true, false);
    }

    // sees if has gone over time
    public boolean isFinished() {
        return timer.get() > Constants.GeneralizedReleaseConstants.shootAnywhereTimeout || optAlliance.isEmpty();
    }

    // sees if the robot is in shooting range
    // goes to the arm angle
    // revs and shoots
    // moves the arm to travel position in the end
    public void end(boolean interrupted) {

        timer.stop();
        boolean isInRange = false;
        if (this.optAlliance.get() == DriverStation.Alliance.Blue) {
            isInRange = (drivetrain.getPose().getX() <= Constants.GeneralizedReleaseConstants.blueShootRange) || (GeometryUtils.isInBlueStage(new OrderedPair(drivetrain.getPose().getX(), drivetrain.getPose().getY()))) || (GeometryUtils.isInRedStage(new OrderedPair(drivetrain.getPose().getX(), drivetrain.getPose().getY())));
        }
        else if (this.optAlliance.get() == DriverStation.Alliance.Red) {
            isInRange = (drivetrain.getPose().getX() >= Constants.GeneralizedReleaseConstants.redShootRange) || (GeometryUtils.isInBlueStage(new OrderedPair(drivetrain.getPose().getX(), drivetrain.getPose().getY()))) || (GeometryUtils.isInRedStage(new OrderedPair(drivetrain.getPose().getX(), drivetrain.getPose().getY())));
        }

        if (!isInRange || !interrupted) {
            intake.stopShoot();
            intake.stopSuck();
            arm.goToAngle(Constants.ArmConstants.kTravelPosition);
            CommandScheduler.getInstance().schedule(DriverCommands.vibrateController(driverController.getHID(), 1));
        } else {
            Command command = OuttakeToSpeaker.shoot(intake).finallyDo(() -> {
                intake.stopShoot();
                intake.stopSuck();
                arm.goToAngle(Constants.ArmConstants.kTravelPosition);
            }); 
            CommandScheduler.getInstance().schedule(command);
        }
    }
}
