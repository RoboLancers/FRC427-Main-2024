package frc.robot.subsystems.hang;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.IOUtils;
import frc.robot.util.MotorSim;
import frc.robot.util.MotorSim.Mode;

public class Hang extends SubsystemBase {

    private static Hang instance = new Hang();

    public static Hang getInstance() {
        return instance; 
    }

    //Initializing velocity variable
    private double m_velocity = 0;
    public double m_targetPosition;
    
    //Initialize Motors
    private MotorSim m_HangMotorRight = new MotorSim(Constants.HangConstants.kHangRightMotorID, MotorType.kBrushless, Mode.MANUAL);
    private MotorSim m_HangMotorLeft = new MotorSim(Constants.HangConstants.kHangLeftMotorID, MotorType.kBrushless, Mode.MANUAL);

    // //Encoder Initialize
    // private RelativeEncoder m_HangMotorRight = m_HangMotorRight.getEncoder();
    // private RelativeEncoder m_HangMotorLeft = m_HangMotorLeft.getEncoder();

    private PIDController m_HangPidController = new PIDController(Constants.HangConstants.kP, Constants.HangConstants.kI, Constants.HangConstants.kD);


    private Hang() {
        setupMotors();
    }

    public void setupMotors() {
        //Sets motors inverted
        m_HangMotorRight.setInverted(Constants.HangConstants.kRightMotorInverted);
        // m_HangMotorLeft.setInverted(Constants.HangConstants.kLeftMotorInverted);
        
        //Sets Smart Limits
        m_HangMotorRight.setSmartCurrentLimit(Constants.HangConstants.kHangMotorLimit);
        m_HangMotorLeft.setSmartCurrentLimit(Constants.HangConstants.kHangMotorLimit);

        //Conversion Factors for left Encoders
        m_HangMotorLeft.setPositionConversionFactor(Constants.HangConstants.kPositionConversionFactor);
        m_HangMotorLeft.setVelocityConversionFactor(Constants.HangConstants.kVelocityConversionFactor);

        //Conversion Factors for right Encoders
        m_HangMotorRight.setPositionConversionFactor(Constants.HangConstants.kPositionConversionFactor);
        m_HangMotorRight.setVelocityConversionFactor(Constants.HangConstants.kVelocityConversionFactor);
        
        //Sets  limits for Right and Left Motors
        //Right Motors
        // m_HangMotorRight.setSoftLimit(SoftLimitDirection.kForward, Constants.HangConstants.kFowardHangSoftLimit);
        // m_HangMotorRight.setSoftLimit(SoftLimitDirection.kReverse, Constants.HangConstants.kReverseHangSoftLimit);
        // m_HangMotorRight.enableSoftLimit(SoftLimitDirection.kForward, true);
        // m_HangMotorRight.enableSoftLimit(SoftLimitDirection.kReverse, true);
        // //Left Motors
        // m_HangMotorLeft.setSoftLimit(SoftLimitDirection.kForward, Constants.HangConstants.kFowardHangSoftLimit);
        // m_HangMotorLeft.setSoftLimit(SoftLimitDirection.kReverse, Constants.HangConstants.kReverseHangSoftLimit);
        // m_HangMotorLeft.enableSoftLimit(SoftLimitDirection.kForward, true);
        // m_HangMotorLeft.enableSoftLimit(SoftLimitDirection.kReverse, true);

        m_HangPidController.setP(Constants.HangConstants.kP);
        m_HangPidController.setI(Constants.HangConstants.kI);
        m_HangPidController.setD(Constants.HangConstants.kD);
        

        // Tells Left Motor to do whatever right motor is doing
        m_HangMotorLeft.follow(m_HangMotorRight, Constants.HangConstants.kLeftMotorInverted);

        // m_HangMotorRight.burnFlash();
        // m_HangMotorLeft.burnFlash();
    }

    @Override
    public void periodic() {
        m_velocity = m_HangPidController.calculate(getHangPosition(), m_targetPosition);
        m_HangMotorRight.set(m_velocity);
       

        m_HangMotorLeft.update(0.02);
        m_HangMotorRight.update(0.02);

        //Check for LEDs on Hang
        

        //Constantly sends logs to Smart Dashboard
        doSendables();
    }

    public void doSendables() {
        // Add logging for hang (eg. encoder positions, velocity, etc. )
        IOUtils.set("Hang Target Velocity (m/s)", m_velocity);
        IOUtils.set("Right Hang Current Velocity (m/s)", m_HangMotorRight.getVelocity());
        IOUtils.set("Left Hang Current Velocity (m/s)", m_HangMotorLeft.getVelocity());
        IOUtils.set("Right Hang Current Position", m_HangMotorRight.getPosition());
        IOUtils.set("Left Hang Current Position", m_HangMotorLeft.getPosition());

        // SmartDashboard.putBoolean("Hang RM Inverted", m_HangMotorRight.getInverted());
        // SmartDashboard.putBoolean("Hang LM Inverted", m_HangMotorLeft.getInverted());
        // IOUtils.set("Hang Soft Limit ForwardRM", m_HangMotorRight.getSoftLimit(SoftLimitDirection.kForward));
        // IOUtils.set("Hang Soft Limit BackRM", m_HangMotorRight.getSoftLimit(SoftLimitDirection.kReverse));
        // IOUtils.set("Hang Soft Limit ForwardLM", m_HangMotorLeft.getSoftLimit(SoftLimitDirection.kForward));
        // IOUtils.set("Hang Soft Limit BackLM", m_HangMotorLeft.getSoftLimit(SoftLimitDirection.kReverse));
        
    }

    public void setSpeed(double speed) {
        //Method to change speed
        this.m_velocity = speed;
    }

    public double getHangPosition() {
        return m_HangMotorRight.getPosition();
    }

    public void setPosition(double targetposition) {
        this.m_targetPosition = targetposition;
    }

    public boolean isAtPosition() {
        return m_HangPidController.atSetpoint();
    }

  
}