package frc.robot.subsystems.drivetrain.commands;

import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.ChassisState;
import frc.robot.util.DriverController;
import frc.robot.util.GeometryUtils;
import frc.robot.util.IOUtils;

public class TeleOpCommand extends Command {
    
    private Drivetrain m_drivetrain; 
    private DriverController m_controller; 

    private FieldState fieldState = FieldState.FIELD; 

    private boolean autoSnap = false; 
    private double autoSnapAngle = 0; 
    private Supplier<Boolean> autoSnapStop = () -> true; 

    public TeleOpCommand(Drivetrain drivetrain, DriverController driverController) {
        this.m_drivetrain = drivetrain; 
        this.m_controller = driverController; 

        addRequirements(this.m_drivetrain);
    } 

    @Override
    public void initialize() {
        if (!SmartDashboard.containsKey("snap")) SmartDashboard.putBoolean("snap", true);
        if (!SmartDashboard.containsKey("Rotation Speed")) SmartDashboard.putNumber("Rotation Speed", 3.14);
        if (!SmartDashboard.containsKey("Linear Speed")) SmartDashboard.putNumber("Linear Speed", 1.0);
        SmartDashboard.putBoolean("Auto Snap At Source", true); 
        m_drivetrain.resetLastTurnedTheta(); 
    }

    @Override
    public void execute() {

        Optional<Alliance> optAlliance = DriverStation.getAlliance(); 

        if (optAlliance.isEmpty()) return; 

        FieldState oldState = this.fieldState; 
        updateFieldState(optAlliance.get());
        // Constants.DrivetrainConstants.kMaxRotationRadPerSecond = SmartDashboard.getNumber("Rotation Speed", 3.14);
        // Constants.DrivetrainConstants.kMaxSpeedMetersPerSecond = SmartDashboard.getNumber("Linear Speed", 1.0);
        // ensure driving does not break if gyro disconnects, will hopefully transition to robot oriented drive
       
        // if (SmartDashboard.getBoolean("snap", true)) {
        //     // align forward, align sideways, etc. 
            ChassisState speeds = m_controller.getDesiredChassisState(); 
            boolean isTurning = speeds.turn; 

            // System.out.println("Speeds turn");
            // System.out.println(speeds.turn);

            if (oldState == FieldState.FIELD && this.fieldState == FieldState.SOURCE) {
                autoSnap = true; 
                autoSnapAngle = optAlliance.get() == Alliance.Red ? 60 : -60; 
                autoSnapStop = () -> Math.abs(m_drivetrain.getRotation().getDegrees() - autoSnapAngle) <= 5 || isTurning || this.fieldState != FieldState.SOURCE; 
            }

            // auto snap
            if (autoSnap && SmartDashboard.getBoolean("Auto Snap At Source", true)) {
                boolean b = autoSnapStop.get(); 
                if (b) {
                    autoSnap = false;
                } else {
                    speeds.turn = true; 
                    speeds.omegaRadians = Math.toRadians(autoSnapAngle); 
                }
            }

            // System.out.println(speeds.omegaRadians);

            SmartDashboard.putNumber("x", speeds.vxMetersPerSecond); 
            SmartDashboard.putNumber("y", speeds.vyMetersPerSecond); 
            SmartDashboard.putNumber("rotation", speeds.omegaRadians); 
            SmartDashboard.putBoolean("Turn", speeds.turn);
            SmartDashboard.putString("Field State", fieldState.name()); 
            m_drivetrain.swerveDriveFieldRel(speeds, true, true);
        // } else {
            // go left go right smoothly
            // ChassisSpeeds speeds = m_controller.getDesiredChassisSpeeds(); 
            // SmartDashboard.putNumber("x", speeds.vxMetersPerSecond); 
            // SmartDashboard.putNumber("y", speeds.vyMetersPerSecond); 
            // SmartDashboard.putNumber("rotation", speeds.omegaRadiansPerSecond); 
            // m_drivetrain.swerveDrive(speeds, true);
        // }
    }

    private void updateFieldState(Alliance alliance) {



        Pose2d robotPose = m_drivetrain.getPose(); 
        boolean isInSource = GeometryUtils.isPoseInRectangle(robotPose, 
        alliance == Alliance.Red ? Constants.Source.redSourceTL : Constants.Source.blueSourceTL, 
        alliance == Alliance.Red ? Constants.Source.redSourceBR : Constants.Source.blueSourceBR); 
        this.fieldState = isInSource ? FieldState.SOURCE : FieldState.FIELD;  
    }

    public enum FieldState {
        FIELD, 
        SOURCE
    }

}