package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.AutomationCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.commands.GoToSpeaker;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.OuttakeToSpeaker;
import frc.robot.commands.RevAndAngle;
import frc.robot.commands.RevAndAngleWithPose;

// class to store, set up, and choose autos
public class AutoPicker {
    private SendableChooser<Command> chooser; 

    Drivetrain m_driveSubsystem;
    
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

        registerCommands(); 

        chooser = AutoBuilder.buildAutoChooser(); 

        // registerExtraAutos();

    }

    public void registerCommands() {
        // eg. NamedCommands.registerCommand("intake_cube", new IntakeForTime(intake, 1, 2)); 
        NamedCommands.registerCommand("GoToSpeaker", new GoToSpeaker(Arm.getInstance()));
        NamedCommands.registerCommand("IntakeGround", AutomationCommands.autoIntakeCommand());
        NamedCommands.registerCommand("ShootSpeaker", new OuttakeToSpeaker());
        NamedCommands.registerCommand("ShootOut", new OuttakeToSpeaker());
        NamedCommands.registerCommand("ShootAnywhere", AutomationCommands.shootFromAnywhere());
        NamedCommands.registerCommand("MoveToNext", new PrintCommand("Moving to next"));
        
        NamedCommands.registerCommand("Shoot", OuttakeToSpeaker.shoot(Intake.getInstance()).finallyDo(() -> Arm.getInstance().goToAngle(Constants.ArmConstants.kTravelPosition)));
        NamedCommands.registerCommand("RevAndAngleAnywhere", new RevAndAngle(Arm.getInstance(), Intake.getInstance(), Drivetrain.getInstance()));
        NamedCommands.registerCommand("RevBlueFirst", RevAndAngleWithPose.createCommand(Arm.getInstance(), Intake.getInstance(), Constants.SetPoints.blueFirstMiddle));
        NamedCommands.registerCommand("RevBlueSecond", RevAndAngleWithPose.createCommand(Arm.getInstance(), Intake.getInstance(), Constants.SetPoints.blueSecondMiddle));
        NamedCommands.registerCommand("RevBlueThird", RevAndAngleWithPose.createCommand(Arm.getInstance(), Intake.getInstance(), Constants.SetPoints.blueThirdMiddle));

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
