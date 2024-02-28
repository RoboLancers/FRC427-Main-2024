package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.commands.ShootAnywhere.ShootAnywhereResult;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.commands.GoToAngle;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.TurnToAngle;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.OuttakeToSpeaker;

public class ShootWithAngle {
     public static Command shootWithAngle(Drivetrain drivetrain, Arm arm, Intake intake, double angle) {
        ShootAnywhereResult res = ShootAnywhere.getShootValues(drivetrain.getPose()); 

        if (res == null) return Commands.none();

        TurnToAngle turnToAngle = new TurnToAngle(drivetrain, res.getDriveAngleDeg());
        GoToAngle goToAngle = new GoToAngle(arm, angle);
        Command outtake = OuttakeToSpeaker.outtakeToSpeaker(intake);
        return Commands.sequence(Commands.parallel(turnToAngle, goToAngle), outtake)
        .finallyDo(() -> {
            arm.goToAngle(Constants.ArmConstants.kTravelPosition);
        });
    }
}
