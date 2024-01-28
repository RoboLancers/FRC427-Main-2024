package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeFromGround extends Command {
     // declare to intake with speed
    Intake m_intake;
    double m_speed;

     // establishes intake, speed
    public IntakeFromGround(Intake intake, double speed) {
        this.m_intake = intake;
        this.m_speed = speed;

        addRequirements(intake);
    }

    // starts intaking (using the sucker motor)
    public void initialize() {
        // runs when the command is FIRST STARTED
    }

    // keeps intaking with the sucker
    public void execute() {
        // runs repeatedly until the command is finished
        this.m_intake.intakeRing(m_speed);
    }

    // checks to stops sucking
    public boolean isFinished() {
        // runs and tells whether or not the command should finish
        return m_intake.beamBreakHit();
    }

    // stops sucking
    public void end(boolean interrupted) {
        // runs when the command is ended
        this.m_intake.intakeRing(0);
    }    
}