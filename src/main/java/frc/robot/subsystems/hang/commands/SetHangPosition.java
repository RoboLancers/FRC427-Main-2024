package frc.robot.subsystems.hang.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hang.Hang;

public class SetHangPosition extends Command {
    Hang m_Hang;
    double m_position;
    
    public SetHangPosition(Hang hang, double position) {
        this.m_Hang = hang;
        this.m_position = position;

        //Makes sure only one thing can run on hang at a time
        addRequirements(hang);
    }


    public void initialize() {
        m_Hang.setPosition(m_position);
        // runs when the command is FIRST STARTED
    }

    public void execute() {
        // runs repeatedly until the command is finished
    }

    public boolean isFinished() {
        // runs and tells whether or not the command should finish
        return m_Hang.isAtPosition(); 
    }

    public void end(boolean interrupted) {
        // runs when the command is ended
    }
}
