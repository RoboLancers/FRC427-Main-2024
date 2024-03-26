package frc.robot.commands;

import java.util.Optional;
import java.util.Set; 
import java.util.function.Consumer; 

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.commands.ShootAnywhere.ShootAnywhereResult;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.SetShooterSpeed;

public class RevAndAngleWithDrive extends RevAndAngle {

    public Arm arm;
    public Intake intake;
    public Drivetrain drivetrain; 
    public RevAndAngleWithDrive(Arm arm, Intake intake, Drivetrain drivetrain, Consumer<Rotation2d> rotation) {
        super(arm, intake, rotation);
        this.drivetrain = drivetrain;  

        addRequirements(arm, intake);
    }

    public static Command createCommand(Arm arm, Intake intake, Drivetrain drivetrain, Consumer<Rotation2d> rotationConsumer) {
        return Commands.defer(() -> new RevAndAngleWithDrive(arm, intake, drivetrain, rotationConsumer), Set.of(arm, intake)).andThen(Commands.idle()).finallyDo(() -> 
        rotationConsumer.accept(null)); 
    }
    
    protected Pose2d getTarget() {
        return this.drivetrain.getPose();
    }
} 
