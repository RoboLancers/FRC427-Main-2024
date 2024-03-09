package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;

public class SetShooterSpeed extends Command {
    // declare to outtake and speed
    Intake m_intake;
    double m_speed;

    // establishes outtake, speed,
    public SetShooterSpeed(Intake intake, double speed) {
        this.m_intake = intake;
        this.m_speed = speed;

        addRequirements(intake);
        
    }
    // starts the shooting motors
    public void initialize() {

    }
    
    // keeps shooting motors going
    public void execute() {
        // runs repeatedly until the command is finished 
        this.m_intake.outtakeRing(m_speed);
    }
    //checks to stop shooting (aww shoot!)
    public boolean isFinished() {
        // runs and tells whether or not the command should finish
        return this.m_intake.atDesiredShootSpeed();
    }
    // stops the Shooting motors
    public void end(boolean interrupted) {
        // runs when the command is ended
    }

    public static Command indexNote(Intake intake) {
        return Commands.runOnce(() -> intake.intakeRing(-0.075))
        .alongWith(Commands.waitUntil(() -> !intake.beamBreakHit()))
        .finallyDo(() -> {
            intake.stopSuck(); 
        }); 
    }

    public static Command revAndIndex(Intake intake, double speed) {
        return indexNote(intake).alongWith(new SetShooterSpeed(intake, speed)); 
    }
}
