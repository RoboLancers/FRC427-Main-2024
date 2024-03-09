package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.IOUtils;

public class TuneOuttakeSucker extends Command {
    Intake m_intake;
    double m_speed;

    public TuneOuttakeSucker(Intake intake) {
        this.m_intake = intake;
        

        addRequirements(intake);
    }

    public void initialize() {

        this.m_intake.outtakeRing(m_speed);
    }
    public void execute() {
        this.m_speed = IOUtils.getNumber("Tune_OuttakeSuck_Speed"); // snake case >:(
    }

    public boolean isFinished() {
        return false; 
    }

    public void end(boolean interrupted) {
        this.m_intake.outtakeRing(0);
    }

}
