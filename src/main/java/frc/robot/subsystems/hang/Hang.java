package frc.robot.subsystems.hang;

import com.revrobotics.CANSparkLowLevel.MotorType;

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
    private double m_manualVelocity = 0; 
    public double m_targetPosition = 0;
    
    //Initialize Motors
    private MotorSim m_HangMotor = new MotorSim(Constants.HangConstants.kHangMotorID, MotorType.kBrushless, Mode.MANUAL);

    // //Encoder Initialize
    // private RelativeEncoder m_HangMotorRight = m_HangMotorRight.getEncoder();
    // private RelativeEncoder m_HangMotorLeft = m_HangMotorLeft.getEncoder();

    private PIDController m_HangPidController = new PIDController(Constants.HangConstants.kP, Constants.HangConstants.kI, Constants.HangConstants.kD);

    private HangControlType m_HangControlType = HangControlType.MANUAL;


    private Hang() {
        setupMotors();
    }

    public void setupMotors() {
        //Sets motors inverted
        m_HangMotor.setInverted(Constants.HangConstants.kMotorInverted);
        
        //Sets Smart Limits
        // m_HangMotor.setSmartCurrentLimit(20, Constants.HangConstants.kHangMotorLimit);

        //Conversion Factors for left Encoders
        m_HangMotor.setPositionConversionFactor(Constants.HangConstants.kPositionConversionFactor);
        m_HangMotor.setVelocityConversionFactor(Constants.HangConstants.kVelocityConversionFactor);

        //Conversion Factors for right Encoders
        m_HangMotor.setPositionConversionFactor(Constants.HangConstants.kPositionConversionFactor);
        m_HangMotor.setVelocityConversionFactor(Constants.HangConstants.kVelocityConversionFactor);
        
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
        
        // m_HangMotor.burnFlash();
    }

    public void setPID(double p, double i, double d) {
        m_HangPidController.setP(p);
        m_HangPidController.setI(i);
        m_HangPidController.setD(d);
    }

    @Override
    public void periodic() {

        m_HangMotor.update(0.02);

        double velocity = 0; 

        //Check for LEDs on Hang
        

        //Constantly sends logs to Smart Dashboard
        doSendables();

          if (m_HangControlType == HangControlType.PID) {
            velocity = m_HangPidController.calculate(getHangPosition(), m_targetPosition);
        }
        
        // velocity controlled manually
        else if (m_HangControlType == HangControlType.MANUAL) {
            velocity = m_manualVelocity; 
        }

        m_HangMotor.set(velocity);
    }

    public void doSendables() {
        // Add logging for hang (eg. encoder positions, velocity, etc. )
        // IOUtils.set("Hang Target Velocity (m/s)", m_velocity);
        IOUtils.set("Hang Current Velocity (m/s)", m_HangMotor.getVelocity());
        IOUtils.set("Hang Current Position", m_HangMotor.getPosition());

        // SmartDashboard.putBoolean("Hang RM Inverted", m_HangMotorRight.getInverted());
        // SmartDashboard.putBoolean("Hang LM Inverted", m_HangMotorLeft.getInverted());
        // IOUtils.set("Hang Soft Limit ForwardRM", m_HangMotorRight.getSoftLimit(SoftLimitDirection.kForward));
        // IOUtils.set("Hang Soft Limit BackRM", m_HangMotorRight.getSoftLimit(SoftLimitDirection.kReverse));
        // IOUtils.set("Hang Soft Limit ForwardLM", m_HangMotorLeft.getSoftLimit(SoftLimitDirection.kForward));
        // IOUtils.set("Hang Soft Limit BackLM", m_HangMotorLeft.getSoftLimit(SoftLimitDirection.kReverse));
        
    }
    public void setManualVelocity(double velocity) {
        this.m_manualVelocity = velocity;
    }

    public double getHangPosition() {
        return m_HangMotor.getPosition();
    }

    public void setPosition(double targetPosition) {
        this.m_targetPosition = targetPosition;
    }

    public boolean isAtPosition() {
        return getError() <= Constants.HangConstants.kHangTolerance; 
    }

    public double getError() {
        return Math.abs(m_HangMotor.getPosition() - this.m_targetPosition); 
    }

    public double getHangVelocity() {
        return m_HangMotor.getVelocity();
    }

    public double getMotorCurrent() {
        return 0; 
        // return m_HangMotor.getOutputCurrent();
    }

    public void setHangMode(HangControlType type) {
        this.m_HangControlType = type;
    }

    public enum HangControlType {
        MANUAL, 
        PID
    }
}