package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmControlType;
import frc.robot.util.IOUtils;

public class TunePIDGoToAngle extends Command {
    Arm m_arm;

    public TunePIDGoToAngle(Arm arm) {
        m_arm = arm;
        addRequirements(arm);
    }

    public void initialize() {
        m_arm.setControlType(ArmControlType.PID);
    }

    public void execute() {
        m_arm.setPID(IOUtils.get("Arm P", Constants.ArmConstants.kP), 
                    IOUtils.get("Arm I", Constants.ArmConstants.kI), 
                    IOUtils.get("Arm D", Constants.ArmConstants.kD));

        m_arm.m_armFeedforward = new ArmFeedforward(
            IOUtils.get("Arm kS", Constants.ArmConstants.kS), 
            IOUtils.get("Arm kG", Constants.ArmConstants.kG), 
            IOUtils.get("Arm kV", Constants.ArmConstants.kV), 
            IOUtils.get("Arm kA", Constants.ArmConstants.kA));
        // m_arm.setKG(IOUtils.get("Arm kG", Constants.ArmConstants.kGravityFF));
        // m_arm.setKS(IOUtils.get("Arm kS", Constants.ArmConstants.kSpringFF));

        m_arm.goToAngle(IOUtils.get("Arm Angle"));
    }

    public boolean isFinished() {
        // confirmation that arm is at angle
        return false; // m_arm.isAtAngle(); 
    }

    public void end(boolean interrupted) {
        // runs when the command is ended
    }
}
