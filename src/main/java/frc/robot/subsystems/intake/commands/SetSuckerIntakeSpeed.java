package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class SetSuckerIntakeSpeed extends Command {
     // declare how long to intake for and speed
    Intake m_intake;
    double m_speed;

     // establishes intake, speed,
    public SetSuckerIntakeSpeed(Intake intake, double speed) {
        this.m_intake = intake;
        this.m_speed = speed;

        addRequirements(intake);
    }

    public void initialize() {}

    // keeps sucking motor going
    public void execute() {
        // runs repeatedly until the command is finished
        this.m_intake.intakeRing(m_speed);
    }

    // checks to stops sucking
    public boolean isFinished() {
        // runs and tells whether or not the command should finish
        return true;
    }

    // stops sucking motor
    public void end(boolean interrupted) {
        // runs when the command is ended
  
    }
}