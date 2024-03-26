package frc.robot.commands;
import java.util.Optional;
import java.util.function.Supplier;
import java.util.function.Consumer; 

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.commands.ShootAnywhere.ShootAnywhereResult;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.SetShooterSpeed;

public abstract class RevAndAngle extends Command {
    public Arm arm;
    public Intake intake;
    public Optional<Alliance> optAlliance;
    private Supplier<Pose2d> robotPoseSupplier; 
    private Consumer<Rotation2d> rotationConsumer; 

    public RevAndAngle(Arm arm, Intake intake, Consumer<Rotation2d> rotation) {
        this.arm = arm;
        this.intake = intake;
        this.robotPoseSupplier = () -> getTarget();
        this.rotationConsumer = rotation; 
  
        addRequirements(arm, intake);
    }

    public void initialize() {
        this.optAlliance = DriverStation.getAlliance();
        CommandScheduler.getInstance().schedule(SetShooterSpeed.indexNote(intake));
    }
    
    public void execute() {
        Pose2d currentPose = this.robotPoseSupplier.get();
        System.out.println(currentPose);
        ShootAnywhereResult res = ShootAnywhere.getShootValues(currentPose); 

        if (res == null) return; 

        arm.goToAngle(res.getArmAngleDeg());
        intake.outtakeRing(res.getOuttakeSpeed());
        rotationConsumer.accept(Rotation2d.fromDegrees(res.getDriveAngleDeg()));
    }

    public void end(boolean interrupted) {
    }

    public boolean isFinished() {
        return optAlliance.isEmpty() || Constants.GeneralizedReleaseConstants.readyToShootAuto.getAsBoolean();
    }

    protected abstract Pose2d getTarget(); 

}