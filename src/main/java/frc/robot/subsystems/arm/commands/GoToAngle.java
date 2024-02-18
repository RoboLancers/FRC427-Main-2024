package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmControlType;

// controlled by pid 
public class GoToAngle extends Command {
    Arm m_arm;
    double m_angle;

    public GoToAngle(Arm arm, double angle) {
        m_arm = arm;
        m_angle = angle;

        addRequirements(arm);
    }

    public void initialize() {
        m_arm.setControlType(ArmControlType.TRAPEZOID);
    }

    public void execute() {
        // makes arm go to angle set point
        m_arm.goToAngle(m_angle);
    }

    public boolean isFinished() {
        // confirmation that arm is at angle
        return m_arm.atGoal(); 
    }

    public void end(boolean interrupted) {
        // runs when the command is ended
    }
}
