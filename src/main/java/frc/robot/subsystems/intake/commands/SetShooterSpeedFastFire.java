package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class SetShooterSpeedFastFire extends Command {
    // declare to outtake and speed
    Intake m_intake;
    double m_speed;

    // establishes outtake, speed,
    public SetShooterSpeedFastFire(Intake intake, double speed) {
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
        return true;
    }
    // stops the Shooting motors
    public void end(boolean interrupted) {
        // runs when the command is ended
        
    }
    
}
