package frc.robot.subsystems.hang.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hang.Hang;
import frc.robot.subsystems.hang.Hang.HangControlType;

public class SetHangSpeed extends Command {
    private Hang m_Hang; 
    private double m_velocity = 0; 

    public SetHangSpeed(Hang hang, double velocity) {
        this.m_Hang = hang; 
        this.m_velocity = velocity; 


        addRequirements(hang);
    }

    
    public void initialize() {
        m_Hang.setHangMode(HangControlType.MANUAL);
        // runs when the command is FIRST STARTED
        m_Hang.setManualVelocity(m_velocity);
    }

    public void execute() {
        // runs repeatedly until the command is finished
    }

    public boolean isFinished() {
        // runs and tells whether or not the command should finish
        return true; 
    }

    public void end(boolean interrupted) {
        // runs when the command is ended
    }
}
