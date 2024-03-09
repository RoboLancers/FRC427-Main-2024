package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ShootAnywhere.ShootAnywhereResult;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.commands.GoToAngle;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.TurnToAngle;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.OuttakeToSpeaker;
import frc.robot.subsystems.leds.Led;
import frc.robot.util.IOUtils;

public class DriverCommands {
  public static Command vibrateController(GenericHID controller, double timeSeconds) {
    return Commands.runOnce(() -> {
      controller.setRumble(RumbleType.kBothRumble, 1);
    }).andThen(new WaitCommand(timeSeconds)).andThen(() -> { 
        controller.setRumble(RumbleType.kBothRumble, 0);
    });  
  }

  public static Command indicateBeamBreak(GenericHID controller) {
    if (DriverStation.isAutonomous()) return Commands.none();
    return Commands.parallel(
      Commands.runOnce(() -> {
        Led.getInstance().beamHit = true; 
      }), 
      vibrateController(controller, 0.5)
    )
    .andThen(new WaitCommand(0.75))
    .finallyDo(() -> Led.getInstance().beamHit = false); 
  }


}
