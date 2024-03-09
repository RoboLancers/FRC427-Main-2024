package frc.robot.commands;

import java.util.Set; 

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ShootAnywhere.ShootAnywhereResult;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.commands.GoToAngle;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.TurnToAngle;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.OuttakeToSpeaker;
import frc.robot.util.IOUtils;

public class TuningCommands {
  public static Command tuneShooting(Drivetrain drivetrain, Arm arm, Intake intake) {
    return Commands.defer(() -> {

        boolean usePresetAngle = IOUtils.getBoolean("TuneShooting/Use Preset Angle");
        boolean usePresetSpeed = IOUtils.getBoolean("TuneShooting/Use Preset Speed"); 

        ShootAnywhereResult res = ShootAnywhere.getShootValues(drivetrain.getPose());
        
        double angle = usePresetAngle ? res.getArmAngleDeg() : IOUtils.getNumber("TuneShooting/Desired Angle"); 
        double speed = usePresetSpeed ? res.getOuttakeSpeed() : IOUtils.getNumber("TuneShooting/Desired Speed");

        if (res == null) return Commands.none();
        TurnToAngle turnToAngle = new TurnToAngle(drivetrain, res.getDriveAngleDeg());

        return Commands.sequence(
            turnToAngle, 
            new GoToAngle(arm, angle),
            Commands.runOnce(() -> {
              drivetrain.swerveDrive(new ChassisSpeeds(), false);
            }),
            OuttakeToSpeaker.revAndIndex(intake, speed).withTimeout(5),
            OuttakeToSpeaker.shoot(intake)
        ).finallyDo(() -> {
          intake.stopShoot();
          intake.stopSuck(); 
        });
    }, Set.of(drivetrain, arm, intake)); 
  }
}
