package frc.robot.commands;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ShootAnywhere.ShootAnywhereResult;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.SetShooterSpeed;

public class RevAndAngle extends Command {
    public Arm arm;
    public Intake intake;
    public Drivetrain drivetrain;
    public Optional<Alliance> optAlliance;

    public RevAndAngle(Arm arm, Intake intake, Drivetrain drivetrain) {
        this.arm = arm;
        this.intake = intake;
        this.drivetrain = drivetrain;
  
        addRequirements(arm, intake);
    }

    public void initialize() {
        this.optAlliance = DriverStation.getAlliance();
        CommandScheduler.getInstance().schedule(SetShooterSpeed.indexNote(intake));
    }
    
    public void execute() {
        Pose2d currentPose = drivetrain.getPose();
        ShootAnywhereResult res = ShootAnywhere.getShootValues(currentPose); 

        if (res == null) return; 

        arm.goToAngle(res.getArmAngleDeg());
        intake.outtakeRing(res.getOuttakeSpeed());
    }

    public boolean isFinished() {
        return optAlliance.isEmpty();
    }

}