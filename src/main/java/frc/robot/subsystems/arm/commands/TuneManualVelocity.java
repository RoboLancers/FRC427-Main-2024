package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmControlType;
import frc.robot.util.IOUtils;

// tuning the best speed that the arm should use
public class TuneManualVelocity extends Command {
    Arm m_arm;

    public TuneManualVelocity(Arm arm) {
        this.m_arm = arm;
        addRequirements(arm);

    }

    public void initialize() {
        m_arm.setControlType(ArmControlType.MANUAL);
    }

    public void execute() {
        m_arm.setSpeed(IOUtils.getNumber("manual speed"));

    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted) {
        
    }
}
