package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.ChassisState;
import frc.robot.util.DriverController;

public class TeleOpCommand extends Command {
    
    private Drivetrain m_drivetrain; 
    private DriverController m_controller; 

    public TeleOpCommand(Drivetrain drivetrain, DriverController driverController) {
        this.m_drivetrain = drivetrain; 
        this.m_controller = driverController; 

        addRequirements(this.m_drivetrain);
    } 

    @Override
    public void initialize() {
        if (!SmartDashboard.containsKey("snap")) SmartDashboard.putBoolean("snap", true);
        m_drivetrain.resetLastTurnedTheta(); 
        if (!SmartDashboard.containsKey("Rotation Speed")) SmartDashboard.putNumber("Rotation Speed", 3.14);
        if (!SmartDashboard.containsKey("Linear Speed")) SmartDashboard.putNumber("Linear Speed", 1.0);
    }

    @Override
    public void execute() {
        // Constants.DrivetrainConstants.kMaxRotationRadPerSecond = SmartDashboard.getNumber("Rotation Speed", 3.14);
        // Constants.DrivetrainConstants.kMaxSpeedMetersPerSecond = SmartDashboard.getNumber("Linear Speed", 1.0);
        // ensure driving does not break if gyro disconnects, will hopefully transition to robot oriented drive
       
        // if (SmartDashboard.getBoolean("snap", true)) {
        //     // align forward, align sideways, etc. 
        //     ChassisState speeds = m_controller.getDesiredChassisState(); 
        //     SmartDashboard.putNumber("x", speeds.vxMetersPerSecond); 
        //     SmartDashboard.putNumber("y", speeds.vyMetersPerSecond); 
        //     SmartDashboard.putNumber("rotation", speeds.omegaRadians); 
        //     SmartDashboard.putBoolean("Turn", speeds.turn);
        //     m_drivetrain.swerveDriveFieldRel(speeds);
        // } else {
            // go left go right smoothly
            ChassisSpeeds speeds = m_controller.getDesiredChassisSpeeds(); 
            SmartDashboard.putNumber("x", speeds.vxMetersPerSecond); 
            SmartDashboard.putNumber("y", speeds.vyMetersPerSecond); 
            SmartDashboard.putNumber("rotation", speeds.omegaRadiansPerSecond); 
            m_drivetrain.swerveDrive(speeds, true);
        // }
    }

}
