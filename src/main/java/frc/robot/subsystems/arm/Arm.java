package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 
public class Arm extends SubsystemBase {
    
    private static Arm instance = new Arm(); 

    public static Arm getInstance() {
        return instance; 
    }
    
    // Define and initiate variables for arm
    private double m_targetPosition = Constants.ArmConstants.kTravelPosition;
    private double m_manualVelocity = 0;
    
    private DigitalInput m_limitSwitch = new DigitalInput(Constants.ArmConstants.kLimitSwitchId);

    private CANSparkMax m_armMotorRight = new CANSparkMax(Constants.ArmConstants.kArmMotorRightId, MotorType.kBrushless);
    private CANSparkMax m_armMotorLeft = new CANSparkMax(Constants.ArmConstants.kArmMotorLeftId, MotorType.kBrushless);

    private AbsoluteEncoder m_armEncoderRight = m_armMotorRight.getAbsoluteEncoder(Type.kDutyCycle);
    private RelativeEncoder m_armRelativeEncoder = m_armMotorRight.getEncoder();
    
    // private ProfiledPIDController m_armProfiledPIDController = new ProfiledPIDController(
    //     Constants.ArmConstants.kP, Constants.ArmConstants.kI, Constants.ArmConstants.kD, 
    //     new TrapezoidProfile.Constraints(Constants.ArmConstants.kMaxArmVelocity, Constants.ArmConstants.kMaxArmAcceleration));

    private PIDController m_armPIDController = new PIDController(Constants.ArmConstants.kP, Constants.ArmConstants.kI, Constants.ArmConstants.kD);
    public ArmFeedforward m_armFeedforward = new ArmFeedforward(Constants.ArmConstants.ksVolts, Constants.ArmConstants.kgVolts, Constants.ArmConstants.kvVoltSecondsPerMeter, Constants.ArmConstants.kaVoltSecondsSquaredPerMeter);
    
    private TrapezoidProfile m_armTrapezoidProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            Constants.ArmConstants.kMaxArmVelocity, Constants.ArmConstants.kMaxArmAcceleration
        )
    );
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State(m_targetPosition, 0);

    private ArmControlType m_ArmControlType = Arm.ArmControlType.TRAPEZOID;

    private Arm() {
        setupEncoders();
        setupController();
        setupMotors();
    }

    // motor and encoder config
    public void setupMotors() {

        m_armMotorRight.setIdleMode(IdleMode.kBrake);
        m_armMotorLeft.setIdleMode(IdleMode.kBrake);

        m_armMotorRight.setInverted(Constants.ArmConstants.kRightMotorInverted);
        // m_armMotorLeft.setInverted(Constants.ArmConstants.kLeftMotorInverted);
        
        m_armMotorRight.setSmartCurrentLimit(Constants.ArmConstants.kMotorCurrentLimit);
        m_armMotorLeft.setSmartCurrentLimit(Constants.ArmConstants.kMotorCurrentLimit);
        
        // left arm motor would follow right arm  motor's voltage intake 
        m_armMotorLeft.follow(m_armMotorRight, Constants.ArmConstants.kLeftMotorInverted);

        m_armMotorLeft.burnFlash(); 
        m_armMotorRight.burnFlash();
    }

    public void setupController() {
        // position error on which it is tolerable
        // m_armProfiledPIDController.setTolerance(Constants.ArmConstants.kTolerance);
        m_armPIDController.setTolerance(Constants.ArmConstants.kTolerance);
    }

    public void setupEncoders() {
        // conversion factors
        m_armEncoderRight.setPositionConversionFactor(Constants.ArmConstants.kPositionConversionFactor);
        m_armEncoderRight.setVelocityConversionFactor(Constants.ArmConstants.kVelocityConversionFactor);

        m_armRelativeEncoder.setPositionConversionFactor(Constants.ArmConstants.kRelativePositionConversionFactor);
        m_armRelativeEncoder.setVelocityConversionFactor(Constants.ArmConstants.kRelativeVelocityConversionFactor);
        
        m_armRelativeEncoder.setPosition(m_armEncoderRight.getPosition());
    }

    public void periodic() {
        doSendables();
        
        double impendingVelocity = 0; 

        if (m_ArmControlType == ArmControlType.TRAPEZOID) {

            // TODO: check Ivan's code !!!

            // impendingVelocity = m_armProfiledPIDController.calculate(getAngle(), m_targetPosition) 
            //                     + m_armFeedforward.calculate(Math.toRadians(getAngle()), 0);

            m_setpoint = m_armTrapezoidProfile.calculate(Constants.ArmConstants.kDt, m_setpoint, m_goal);

            impendingVelocity = m_armPIDController.calculate(getAngle(), m_setpoint.position)
                                + m_armFeedforward.calculate(Math.toRadians(getAngle()), 0);                    
        }
            
        // velocity controlled manually
        else if (m_ArmControlType == ArmControlType.MANUAL) {
            impendingVelocity = m_manualVelocity;
        }

        // if the arm is not reaching beyond the limit switch,
        // or if the arm is not reaching beyond 100 degs, 
        // arm is allowed to move 
        // forward defined as away from the limit switch.
        // boolean passReverseSoftLimit = reverseSoftLimit() && impendingVelocity < 0;
        // boolean passForwardSoftLimit = forwardSoftLimit() && impendingVelocity > 0;

        // if (!passReverseSoftLimit && !passForwardSoftLimit) {
            m_armMotorRight.set(impendingVelocity); 
        // }

        SmartDashboard.putNumber("Impending Velocity (m/s)", impendingVelocity);
    }

    public boolean reverseSoftLimit() {
        // return (getLimitSwitchValue() || getAngle() < Constants.ArmConstants.kReverseSoftLimit);
        return (getAngle() < Constants.ArmConstants.kReverseSoftLimit);
    }

    public boolean forwardSoftLimit() {
        return getAngle() > Constants.ArmConstants.kForwardSoftLimit;
    }

    public boolean getLimitSwitchValue() {
        return m_limitSwitch.get(); 
    }

    public double getError() {
        return m_armPIDController.getPositionError();
    }

    public double getVelocityError() {
        return m_armPIDController.getVelocityError();
    }

    public void goToAngle(double angle) {
        this.m_targetPosition = angle;
    }

    public double getAngle() {
        return m_armRelativeEncoder.getPosition() > 180 ? m_armRelativeEncoder.getPosition() - 360 : m_armRelativeEncoder.getPosition();
    }

    public void setSpeed(double speed) {
        this.m_manualVelocity = speed;
    }

    public boolean atSetpoint() {
        return m_armPIDController.atSetpoint();
    }

    public boolean atGoal() {
        return atSetpoint() && m_setpoint.equals(m_goal);
        
    }

    public void setControlType(ArmControlType type) {
        this.m_ArmControlType = type;
    }

    public void setPID(double p, double i, double d) {
        this.m_armPIDController.setPID(p, i, d);
    }

    public ArmControlState getArmControlState() {
        if (m_targetPosition == Constants.ArmConstants.kGroundPosition) {
            return ArmControlState.GROUND;
        }
        else if (m_targetPosition == Constants.ArmConstants.kTravelPosition) {
            return ArmControlState.TRAVEL;
        }
        else if (m_targetPosition == Constants.ArmConstants.kAmpPosition) {
            return ArmControlState.AMP;
        }
        else if (m_targetPosition == Constants.ArmConstants.kSpeakerPosition) {
            return ArmControlState.SPEAKER;
        }
        else {
            return ArmControlState.CUSTOM;
        }
    }

    public enum ArmControlType {
        MANUAL, 
        PID, 
        TRAPEZOID
    }

    public enum ArmControlState {
        GROUND,
        TRAVEL,
        AMP,
        SPEAKER, 
        CUSTOM
    }

    // add logging for arm 
    // are units correct?
    public void doSendables() {
        SmartDashboard.putNumber("Arm Target Position (deg)", m_targetPosition);
        SmartDashboard.putNumber("Arm Position (deg)", getAngle()); 
        SmartDashboard.putNumber("Arm Velocity (deg/sec)", m_armEncoderRight.getVelocity());
        SmartDashboard.putNumber("Arm Position Error (deg)", getError());
        SmartDashboard.putNumber("Arm Velocity Error (deg/sec)", getVelocityError());
        SmartDashboard.putBoolean("Is Arm At Set Point", atSetpoint());
        SmartDashboard.putBoolean("Is Arm At Goal", atGoal());
        SmartDashboard.putString("Arm Control Type", m_ArmControlType.toString());
        SmartDashboard.putString("Arm Control State", getArmControlState().toString());
        SmartDashboard.putNumber("left volt", m_armMotorLeft.get()); 
        SmartDashboard.putNumber("right volt", m_armMotorRight.get()); 

        // SmartDashboard.putBoolean("Arm Limit Switch", getLimitSwitchValue());
        // SmartDashboard.putBoolean("left inverted", m_armMotorLeft.getInverted()); 
        // SmartDashboard.putBoolean("right inverted", m_armMotorRight.getInverted()); 
    }
}
