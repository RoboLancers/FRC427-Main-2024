package frc.robot.subsystems.hang.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hang.Hang;
import frc.robot.subsystems.hang.Hang.HangControlType;
import frc.robot.util.IOUtils;

//Fine optimal hang Speed
public class TuneSetHangPosition extends Command {
    //Create Hang and Speed
    private Hang m_Hang;
    
    public TuneSetHangPosition(Hang hang) {
        m_Hang = hang;
        
        //Makes sure only one thing can run on hang at a time
        addRequirements(hang);
    }


    public void initialize() {
        m_Hang.setHangMode(HangControlType.PID);
        // runs when the command is FIRST STARTED
    }

    public void execute() {
        //Gets speed from IOUtils
        m_Hang.setPID(IOUtils.getNumber("Hang kP"), IOUtils.getNumber("Hang kI"), IOUtils.getNumber("Hang kD"));
        m_Hang.setPosition(IOUtils.getNumber("Hang Target Position"));
        // runs repeatedly until the command is finished
    }

    public boolean isFinished() {
        // runs and tells whether or not the command should finish
        return false; 
    }

    public void end(boolean interrupted) {
        // runs when the command is ended
    }
}
