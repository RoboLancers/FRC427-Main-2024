// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set; 

import frc.robot.commands.AutoHang;
import frc.robot.commands.AutomaticallyMoveToPiece;
import frc.robot.commands.AutomationCommands;
import frc.robot.commands.DriverCommands;
import frc.robot.commands.TuningCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmControlState;
import frc.robot.subsystems.arm.commands.GoToAmp;
import frc.robot.subsystems.arm.commands.GoToAngle;
import frc.robot.subsystems.arm.commands.GoToGround;
import frc.robot.subsystems.arm.commands.GoToSpeaker;
import frc.robot.subsystems.arm.commands.GoToTravel;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.TeleOpCommand;
import frc.robot.subsystems.hang.Hang;
import frc.robot.subsystems.hang.commands.SetHangSpeed;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeFromGround;
import frc.robot.subsystems.intake.commands.OuttakeToAmp;
import frc.robot.subsystems.intake.commands.OuttakeToSpeaker;
import frc.robot.util.DriverController;
import frc.robot.util.IOUtils;
import frc.robot.util.DriverController.Mode;
import frc.robot.subsystems.leds.Led;
import frc.robot.subsystems.vision.FrontVision;
import frc.robot.subsystems.vision.Vision_old;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
  private final AutoPicker autoPicker; 

  // drivetrain of the robot
  private final Drivetrain drivetrain = Drivetrain.getInstance();

  // intake of the bot
  private final Intake intake = Intake.getInstance(); 

  // private final Vision_old vision = Vision_old.getInstance();

  // leds!
  private final Led led = Led.getInstance(); 
  // private final AddressableLEDSim sim = new AddressableLEDSim(led.getLED()); 

  // limelight subsystem of robot
  // private final BackVision backVision = BackVision.getInstance();
  // private final FrontVision frontVision = FrontVision.getInstance(); 

  // hang mechanism of robot
  // private final Hang hang = Hang.getInstance();
  
  // arm of the robot
  private final Arm arm = Arm.getInstance();
  
  // private SendableChooser<LEDPattern> patterns = new SendableChooser<>();

  // controller for the driver
  private final DriverController driverController =
      DriverController.getInstance(); 

  private final CommandXboxController manipulatorController = new CommandXboxController(Constants.OperatorConstants.kManipulatorControllerPort); 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoPicker = new AutoPicker(drivetrain); 
    // Configure the trigger bindings
    configureBindings();

    // driverController.setChassisSpeedsSupplier(drivetrain::getChassisSpeeds); // comment in simulation
    // default command for drivetrain is to calculate speeds from controller and drive the robot
    drivetrain.setDefaultCommand(new TeleOpCommand(drivetrain, driverController));
  }


  private void configureBindings() {

    new Trigger(() -> DriverStation.getMatchTime() < 21 && DriverStation.getMatchTime() > 19)
    .onTrue(DriverCommands.vibrateController(driverController.getHID(), 2).alongWith(DriverCommands.vibrateController(manipulatorController.getHID(), 2)));

    // --- Driver ---

    driverController.b().onTrue(new InstantCommand(() -> {
      Optional<Alliance> alliance = DriverStation.getAlliance(); 

      if (alliance.isEmpty()) return; 


      drivetrain.setHeading(Rotation2d.fromDegrees(alliance.get() == Alliance.Red ? 180 : 0)); 
    }));

    // driverController.rightBumper()
    //   .onTrue(new InstantCommand(() -> driverController.setSlowMode(Mode.SLOW)))
      // .onFalse(new InstantCommand(() -> driverController.setSlowMode(Mode.NORMAL))); 
    
    // driverController.y()
    // .whileTrue(AutomationCommands.generalizedHangCommand(driverController));


    manipulatorController.rightTrigger().onTrue(Commands.runOnce(() -> {
      intake.intakeRing(-0.2);
    })).onFalse(Commands.runOnce(() -> {
      intake.stopSuck();
    }));

    manipulatorController.rightBumper()
    .whileTrue(Commands.parallel(OuttakeToSpeaker.revAndIndex(intake, 3000, 3000), new GoToAngle(arm, 12)))
    .onFalse(
      OuttakeToSpeaker.shoot(intake, 0.5)
      .andThen(Commands.runOnce(() -> {
        arm.goToAngle(Constants.ArmConstants.kTravelPosition);
      }))
    ); 

    // driverController.y()
    // .whileTrue(Commands.defer(() -> Commands.parallel(OuttakeToSpeaker.revAndIndex(intake, IOUtils.get("TuneShot/ShotSpeedTop", 0), IOUtils.get("TuneShot/ShotSpeedBottom", 0)), new GoToAngle(arm, IOUtils.get("TuneShot/ShotAngle", 0))), Set.of(arm, intake)))
    // .onFalse(
    //   OuttakeToSpeaker.shoot(intake, 0.5)
    //   .andThen(Commands.runOnce(() -> {
    //     arm.goToAngle(Constants.ArmConstants.kTravelPosition);
    //   }))
    // ); 

    // TODO: tune
    // driverController.y().whileTrue(TuningCommands.tuneShooting(drivetrain, arm, intake)); 

    // TODO: tune
    // driverController.y().whileTrue(new TuneTurnToAngle(drivetrain)); 

    // --- Intake --- 

    // outtake
    manipulatorController.leftBumper().and(() -> arm.getArmControlState() == ArmControlState.AMP)
      .whileTrue(new OuttakeToAmp(intake).finallyDo(() -> {
        intake.stopSuck(); 
        intake.stopShoot();
      }));

    // hold a button to rev up, outtakes after release
    manipulatorController.leftBumper().and(() -> arm.getArmControlState() == ArmControlState.SPEAKER)
    .whileTrue(OuttakeToSpeaker.revAndIndex(intake, 2400, 2400))
    .onFalse(OuttakeToSpeaker.shoot(intake, 0.5));
    
    // intake
    manipulatorController.leftBumper().and(() -> arm.getArmControlState() == ArmControlState.GROUND)
    .whileTrue(new IntakeFromGround(intake));

    // driverController.rightTrigger()
    // .whileTrue(AutomationCommands.autoIntakeCommand()); // intake from ground auto

    // driverController.x()
    // .whileTrue(AutomationCommands.pathFindToGamePiece(driverController)); 

    // driverController.a()
    // .whileTrue(AutomationCommands.pathFindToAmpAndMoveArm());

    // driverController.leftTrigger()
    // .whileTrue(AutomationCommands.generalizedReleaseCommand(driverController));



    // arm setpoints
    manipulatorController.a().onTrue(new GoToGround(arm));
    manipulatorController.b().onTrue(new GoToTravel(arm));
    manipulatorController.x().onTrue(new GoToSpeaker(arm));
    manipulatorController.y().onTrue(new GoToAmp(arm));


    // --- Hang ---

    // Hang Up when DPAD UP
    // new Trigger(() -> manipulatorController.getLeftY() >= 0.5)
    //   .onTrue(new SetHangSpeed(hang, Constants.HangConstants.kHangSpeed)); 

    // new Trigger(() -> manipulatorController.getLeftY() <= -0.5)
    //   .onTrue(new SetHangSpeed(hang, -Constants.HangConstants.kHangSpeed)); 



    // Stop hang when neither is pressed
    // new Trigger(() -> 
    //   manipulatorController.getLeftY() < 0.5 && 
    //   manipulatorController.getLeftY() > -0.5)
    // .onTrue(new SetHangSpeed(hang, 0)); 
    



    // TESTING
    // driverController.y().onTrue(Commands.runOnce(() -> hang.setSpeed(0.6))).onFalse(Commands.runOnce(() -> hang.setSpeed(0)));
    // driverController.y().onTrue(Commands.runOnce(() -> arm.setSpeed(0.1))).onFalse(Commands.runOnce(() -> arm.setSpeed(0)));  
    // driverController.b().onTrue(Commands.runOnce(() -> hang.setSpeed(-0.6))).onFalse(Commands.runOnce(() -> hang.setSpeed(0)));
    // driverController.y().onTrue(Commands.runOnce(() -> hang.setSpeed(0.))).onFalse(Commands.runOnce(() -> hang.setSpeed(0))); 
    // driverController.x().onTrue(new SetVelocity(arm, -0.4)).onFalse(new SetVelocity(arm, 0));
    // manipulatorController.x().onTrue(new SetSuckerIntakeSpeed(intake, -0.5)).onFalse(new SetSuckerIntakeSpeed(intake, 0)); 
    // manipulatorController.y().onTrue(SetShooterSpeed.revAndIndex(intake, 1)).onFalse(new SetShooterSpeed(intake, 0));

  }
  

  // send any data as needed to the dashboard
  public void doSendables() {
    SmartDashboard.putData("Autonomous", autoPicker.getChooser());
    SmartDashboard.putBoolean("gyro connected", drivetrain.gyro.isConnected()); 
  }

  // gives the currently picked auto as the chosen auto for the match
  public Command getAutonomousCommand() {
      // return null; 
    return autoPicker.getAuto();
    // return intake.tuneBottomPID(); 
    // return tunerCommand;

  }
}
