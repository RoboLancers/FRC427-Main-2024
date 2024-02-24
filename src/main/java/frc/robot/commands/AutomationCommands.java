package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.commands.GoToAmp;
import frc.robot.subsystems.arm.commands.GoToGround;
import frc.robot.subsystems.arm.commands.GoToSpeaker;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.MoveToAmp;
import frc.robot.subsystems.drivetrain.commands.MoveToSpeaker;
import frc.robot.subsystems.hang.Hang;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeFromGround;
import frc.robot.subsystems.intake.commands.OuttakeToAmp;
import frc.robot.subsystems.intake.commands.OuttakeToSpeaker;
import frc.robot.subsystems.leds.Led;
import frc.robot.subsystems.vision.FrontVision;
import frc.robot.util.DriverController;

public class AutomationCommands {
  
  //Create autoIntakeCOmmand Command
  public static Command autoIntakeCommand() {
    // Set Leds to Intake
    return Commands.runOnce(() -> Led.getInstance().isIntaking = true).andThen(Commands.parallel(
        new IntakeFromGround(Intake.getInstance()),
        new GoToGround(Arm.getInstance())
      )).finallyDo(() -> {
        // Reset arm to travel and reset Leds
        Arm.getInstance().goToAngle(Constants.ArmConstants.kTravelPosition);
        Led.getInstance().isIntaking = false;
    });
  }

  //Create pathFindToSpeaker Command
  public static Command pathFindToSpeaker() {
    //Sets leds
    return Commands.runOnce(() -> Led.getInstance().isMovingToSpeaker = true).andThen(
      Commands.defer(() -> MoveToSpeaker.goToSpeaker(), Set.of(Drivetrain.getInstance()))).finallyDo(() -> {
        //Set leds off 
        Led.getInstance().isMovingToSpeaker = false;
      }); 
  }

  //Create pathFindToSpeakerAndScore Command
  public static Command pathFindToAmp() {
    //Set Leds
    return Commands.runOnce(() -> Led.getInstance().isMovingToAmp = true).andThen(
      //Go To Amp
      Commands.defer(() -> MoveToAmp.goToAmp(), Set.of(Drivetrain.getInstance()))).finallyDo(() -> {
        //Turn specific Leds off
        Led.getInstance().isMovingToAmp = false;
      }); 
  }

  //Create pathFindToSpeakerAndScore Command
  public static Command pathFindToSpeakerAndScore() {
    return pathFindToSpeaker().alongWith(new GoToSpeaker(Arm.getInstance())).andThen(OuttakeToSpeaker.outtakeToSpeaker(Intake.getInstance())).finallyDo(() -> {
      Arm.getInstance().goToAngle(Constants.ArmConstants.kTravelPosition);
    }); 
  }
  //Create pathFindToAmpAnndScore Command
  public static Command pathFindToAmpAndScore() {
    //Run pathFindToAmp
    return pathFindToAmp().alongWith(new GoToAmp(Arm.getInstance())).andThen(new OuttakeToAmp(Intake.getInstance())).finallyDo(() -> {
      //When done, arm goes to travel position
      Intake.getInstance().stopShoot();
      Intake.getInstance().stopSuck();
      Arm.getInstance().goToAngle(Constants.ArmConstants.kTravelPosition);
    });
  }

  public static Command pathFindToGamePiece(DriverController controller) {
    return Commands.runOnce(() -> {
      Led.getInstance().isMovingToNote = true; 
    }).andThen(Commands.defer(
        () -> AutomaticallyMoveToPiece.automaticallyMoveToPiece(controller, Drivetrain.getInstance(), FrontVision.getInstance()), 
        Set.of(Drivetrain.getInstance(), FrontVision.getInstance())
      )
    ).finallyDo(() -> {
      Led.getInstance().isMovingToNote = false; 
    }); 
  }


  //Create ShootFromAnywhere Commands
  public static Command shootFromAnywhere() {
    //When command runs set Leds to true, and then run Shoot Anywhere with Drivetrain/Arm/Intake
    return Commands.runOnce(() -> Led.getInstance().isShooting = true).andThen(Commands.defer(
      () -> ShootAnywhere.shootAnywhere(Drivetrain.getInstance(), Arm.getInstance(), Intake.getInstance()), 
    Set.of(Drivetrain.getInstance(), Arm.getInstance(), Intake.getInstance()))).finallyDo(() -> {
      //Set Is Shooting to False
      Led.getInstance().isShooting = false;
    }); 
  }

  public static Command generalizedReleaseCommand(DriverController controller) {
    return Commands.runOnce(() -> Led.getInstance().isShooting = true).andThen(
      new GeneralizedReleaseRoutine(controller, Drivetrain.getInstance(), Arm.getInstance(), Intake.getInstance())
    ).finallyDo(() -> {
      Led.getInstance().isShooting = false; 
    }); 
  }

  public static Command generalizedHangCommand(DriverController controller) {
    return Commands.runOnce(() -> Led.getInstance().isHanging = true)
    .andThen(new GeneralizedHangRoutine(controller, Drivetrain.getInstance(), Hang.getInstance()))
    .finallyDo(() -> {
      Led.getInstance().isHanging = false; 
    });
  }
}
