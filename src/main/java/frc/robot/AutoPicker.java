package frc.robot;

import java.util.Optional;
import java.util.function.Consumer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.AutomationCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.commands.GoToAngle;
import frc.robot.subsystems.arm.commands.GoToSpeaker;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.OuttakeToSpeaker;
import frc.robot.subsystems.intake.commands.SetShooterSpeed;
import frc.robot.commands.RevAndAngle;
import frc.robot.commands.RevAndAngleWithPose;

// class to store, set up, and choose autos
public class AutoPicker {
    private SendableChooser<Command> chooser; 

    Drivetrain m_driveSubsystem;

    private Rotation2d rotationOverride = null; 
    
    public AutoPicker(Drivetrain driveSubsystem) {
        m_driveSubsystem = driveSubsystem;

        // addRequirements(driveSubsystem);

        // see PathPlanner

        AutoBuilder.configureHolonomic(
            driveSubsystem::getPose, // Pose2d supplier
            driveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            driveSubsystem::getChassisSpeeds, // current robot-relative speeds for the drivetrain
            driveSubsystem::swerveDriveWithoutCompensation, // Module states consumer used to output to the drive subsystem
            new HolonomicPathFollowerConfig(
                new PIDConstants(Constants.Trajectory.kDrive_P, Constants.Trajectory.kDrive_I, Constants.Trajectory.kDrive_D), // PID constants to correct for translation error (used to create the X and Y PID controllers)
                new PIDConstants(Constants.Trajectory.kOmega_P, Constants.Trajectory.kOmega_I, Constants.Trajectory.kOmega_D), 
                Constants.DrivetrainConstants.kMaxAttainableModuleSpeedMetersPerSecond, Constants.DrivetrainConstants.kDriveBaseRadius, 
                new ReplanningConfig(false, false)
            ),  
            () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                }, 
            driveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
        );

        PPHolonomicDriveController.setRotationTargetOverride(() -> {
            System.out.println(this.rotationOverride);
            return this.rotationOverride == null ? Optional.empty() : Optional.of(this.rotationOverride); 
        });

        registerCommands(); 

        chooser = AutoBuilder.buildAutoChooser(); 

        // registerExtraAutos();

    }

    public void registerCommands() {

        Consumer<Rotation2d> rotationConsumer = (rot) -> this.rotationOverride = rot; 

        // eg. NamedCommands.registerCommand("intake_cube", new IntakeForTime(intake, 1, 2)); 
        NamedCommands.registerCommand("GoToSpeaker", new GoToSpeaker(Arm.getInstance()));
        NamedCommands.registerCommand("IntakeGround", AutomationCommands.autoIntakeCommand(0.5).withTimeout(3));
        NamedCommands.registerCommand("ShootSpeaker", OuttakeToSpeaker.outtakeToSpeaker(Intake.getInstance()));
        // NamedCommands.registerCommand("ShootOut", new OuttakeToSpeaker(Intake.getInstance(),0.5,1));
        NamedCommands.registerCommand("ShootAnywhere", AutomationCommands.shootFromAnywhere());
        NamedCommands.registerCommand("MoveToNext", new PrintCommand("Moving to next"));
        
        NamedCommands.registerCommand("Shoot", OuttakeToSpeaker.shoot(Intake.getInstance()).finallyDo(() -> Arm.getInstance().goToAngle(Constants.ArmConstants.kTravelPosition)));
        NamedCommands.registerCommand("RevAndAngleAnywhere", new RevAndAngle(Arm.getInstance(), Intake.getInstance(), Drivetrain.getInstance()));
        NamedCommands.registerCommand("RevBlueFirst", RevAndAngleWithPose.createCommand(Arm.getInstance(), Intake.getInstance(), Constants.SetPoints.blueFirstMiddle));
        NamedCommands.registerCommand("RevBlueSecond", RevAndAngleWithPose.createCommand(Arm.getInstance(), Intake.getInstance(), Constants.SetPoints.blueSecondMiddle));
        NamedCommands.registerCommand("RevBlueThird", RevAndAngleWithPose.createCommand(Arm.getInstance(), Intake.getInstance(), Constants.SetPoints.blueThirdMiddle));
        NamedCommands.registerCommand("RevBlueCenter", RevAndAngleWithPose.createCommand(Arm.getInstance(), Intake.getInstance(), Constants.SetPoints.blueCenter));
        NamedCommands.registerCommand("RevOut", new GoToAngle(Arm.getInstance(), 20).alongWith(new SetShooterSpeed(Intake.getInstance(), 2400, 2400)));
        NamedCommands.registerCommand("AutomaticallyPickupNote", AutomationCommands.pickupNote());

        NamedCommands.registerCommand("AngleBlueFirst", RevAndAngleWithPose.createRotationCommand(Constants.SetPoints.blueFirstMiddle, rotationConsumer));



        // NamedCommands.registerCommand("GoToSpeaker", new PrintCommand("Going to Speaker"));
        // NamedCommands.registerCommand("IntakeGround", new PrintCommand("Intaking from ground!"));
        // NamedCommands.registerCommand("ShootSpeaker", new PrintCommand("shooting to speaker!!"));
        NamedCommands.registerCommand("ShootOut", new PrintCommand("Shooting out to nowhere!!!"));
        // NamedCommands.registerCommand("ShootAnywhere", new PrintCommand("Shooting from anywhere!!!!"));
    }

    public void registerExtraAutos() {
        chooser.addOption("JustShoot", AutomationCommands.shootFromAnywhere());
    }

    // gets the currently selected auto
    public Command getAuto() {
        return chooser.getSelected(); 
    }

    public SendableChooser<Command> getChooser() {
        return chooser; 
    }
}
