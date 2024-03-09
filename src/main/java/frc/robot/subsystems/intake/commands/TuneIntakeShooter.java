package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.IOUtils;

public class TuneIntakeShooter extends Command {
    Intake m_intake;
    double m_speed;

    public TuneIntakeShooter(Intake intake) {
        this.m_intake = intake;
        

        addRequirements(intake);
    }

    public void initialize() {
        this.m_intake.intakeRing(m_speed);
    }

    public void execute() {
        this.m_speed = IOUtils.getNumber("Tune_IntakeShoot_Speed");
    }

    public boolean isFinished() {
        return false; 
    }

    public void end(boolean interrupted) {
        this.m_intake.intakeRing(0);
    }

}
