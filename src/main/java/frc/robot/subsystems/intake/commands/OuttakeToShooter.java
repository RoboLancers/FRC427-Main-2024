package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class OuttakeToShooter extends Command {
     // declare how long to intake for and speed
    Intake m_intake;
    double m_speed;

     // establishes intake, speed,
    public OuttakeToShooter(Intake intake, double speed) {
    this.m_intake = intake;
    this.m_speed = speed;

    addRequirements(intake);
    }

    // starts outtaking 
    public void initialize() {
        // runs when the command is FIRST STARTED
    this.m_intake.outtakeRing(m_speed);
    }

    // keeps outtaking
    public void execute() {
        // runs repeatedly until the command is finished
    this.m_intake.outtakeRing(m_speed);
    
    }

    // checks to stops outtaking
    public boolean isFinished() {
        // runs and tells whether or not the command should finish
        return m_intake.beamBreakHit();
    }

    // stops outtaking
    public void end(boolean interrupted) {
        // runs when the command is ended
    this.m_intake.outtakeRing(0);
    }    
}
