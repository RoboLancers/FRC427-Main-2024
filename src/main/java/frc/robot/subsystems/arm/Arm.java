package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MotorSim;
import frc.robot.util.MotorSim.Mode;
import frc.robot.util.IOUtils;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
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

    private MotorSim m_armMotorRight = new MotorSim(Constants.ArmConstants.kArmMotorRightId, MotorType.kBrushless, Mode.MANUAL);
    private MotorSim m_armMotorLeft = new MotorSim(Constants.ArmConstants.kArmMotorLeftId, MotorType.kBrushless, Mode.MANUAL);

    //private AbsoluteEncoder m_armEncoderRight = m_armMotorRight.getAbsoluteEncoder(Type.kDutyCycle);
    
    private PIDController m_armPIDController = new PIDController(Constants.ArmConstants.kP, Constants.ArmConstants.kI, Constants.ArmConstants.kD);
    
    public ArmFeedforward m_armFeedforward = new ArmFeedforward(Constants.ArmConstants.kS, Constants.ArmConstants.kG, Constants.ArmConstants.kV, Constants.ArmConstants.kA);

    private ArmControlType m_ArmControlType = Arm.ArmControlType.PID;

    // custom arm feedforward with gas springs
    // private double m_kG = Constants.ArmConstants.kGravityFF;
    // private double m_kS = Constants.ArmConstants.kSpringFF;

    StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
    .getStructTopic("Arm Pose", Pose3d.struct).publish();

    private Arm() {
        setupMotors();
    }

    // motor and encoder config
    public void setupMotors() {
        m_armMotorRight.setInverted(false);
        m_armMotorLeft.setInverted(true);
        
        m_armMotorRight.setSmartCurrentLimit(Constants.ArmConstants.kMotorCurrentLimit);
        m_armMotorLeft.setSmartCurrentLimit(Constants.ArmConstants.kMotorCurrentLimit);

        // conversion factors
        m_armMotorRight.setPositionConversionFactor(Constants.ArmConstants.kPositionConversionFactor);
        m_armMotorRight.setVelocityConversionFactor(Constants.ArmConstants.kVelocityConversionFactor);
        
        // position error on which it is tolerable
    //    m_armPIDController.setTolerance(Constants.ArmConstants.kTolerance);
        
        // left arm motor would follow right arm  motor's voltage intake 
        m_armMotorLeft.follow(m_armMotorRight, true);

        // m_armMotorLeft.burnFlash(); 
        // m_armMotorRight.burnFlash();
    }

    public void periodic() {
        m_armMotorRight.update(0.02);
        m_armMotorLeft.update(0.02);
        doSendables();

        
    // publisher.accept(new Pose3d(new Translation3d(IOUtils.get("arm x", 0), IOUtils.get("arm y", 0), IOUtils.get("arm z", 0)), new Rotation3d(IOUtils.get("arm roll", 0) * Math.PI / 180, IOUtils.get("arm pitch", 0) * Math.PI / 180, IOUtils.get("arm yaw", 0) * Math.PI / 180)));
    publisher.accept(new Pose3d(new Translation3d(-0.20447, 0, 0.29), new Rotation3d(90 * Math.PI / 180, getAngle() * Math.PI / 180, 180 * Math.PI / 180)));
        // SmartDashboard.putNumberArray("Arm Pose", new double[] {0, 0, 0, 0, -25, 0}); 
        // -0.20447, 0, 0.29, 90, 0, 180 
        
        double impendingVelocity = 0; 

        if (m_ArmControlType == ArmControlType.PID) {

            impendingVelocity = m_armPIDController.calculate(getAngle(), m_targetPosition) 
                                + m_armFeedforward.calculate(Math.toRadians(getAngle()), 0);
            
            // custom arm feedforward
            // impendingVelocity =  m_armPIDController.calculate(m_armEncoderRight.getPosition(), m_targetPosition) 
            //                  + m_kG * Math.cos(Math.toRadians(m_armEncoderRight.getPosition()))
            //                  + m_kS;
            
        }
        
        // velocity controlled manually
        else if (m_ArmControlType == ArmControlType.MANUAL) {
            impendingVelocity = m_manualVelocity;
        }

        // if the arm is not reaching beyond the limit switch,
        // or if the arm is not reaching beyond 100 degs, 
        // arm is allowed to move 
        // forward defined as away from the limit switch.
        boolean passReverseSoftLimit = reverseSoftLimit() && impendingVelocity < 0;
        boolean passForwardSoftLimit = forwardSoftLimit() && impendingVelocity > 0;

        // if (!passReverseSoftLimit && !passForwardSoftLimit) {
            m_armMotorRight.set(impendingVelocity); 
        // }

        SmartDashboard.putNumber("Impending Velocity (m/s)", impendingVelocity);
        SmartDashboard.putBoolean("Pass Reverse Soft Limit", passReverseSoftLimit);
        SmartDashboard.putBoolean("Pass Forward Soft Limit", passForwardSoftLimit);
    }

    public boolean reverseSoftLimit() {
        return (getAngle() < Constants.ArmConstants.kReverseSoftLimit);
    }

    public boolean forwardSoftLimit() {
        return getAngle() > Constants.ArmConstants.kForwardSoftLimit;
    }

    public boolean getLimitSwitchValue() {
        return m_limitSwitch.get(); 
    }

    // public void setKG(double kG) {
    //     this.m_kG = kG;
    // }

    // public void setKS(double kS) {
    //     this.m_kS = kS;
    // }

    public double getError() {
        return m_armPIDController.getPositionError();
    }

    public void goToAngle(double angle) {
        this.m_targetPosition = angle;
    }

    public double getAngle() {
        return m_armMotorRight.getPosition() > 180 ? m_armMotorRight.getPosition() - 360 : m_armMotorRight.getPosition();
    }


    public void setSpeed(double speed) {
        this.m_manualVelocity = speed;
    }

    public boolean isAtAngle() {
        return m_armPIDController.atSetpoint();
        
    }

    public void setControlType(ArmControlType type) {
        this.m_ArmControlType = type;
    }

    public void setPID(double p, double i, double d) {
        this.m_armPIDController.setPID(p, i, d);
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
        PID
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
        SmartDashboard.putNumber("Arm Velocity (deg/sec)", m_armMotorRight.getVelocity());
        SmartDashboard.putNumber("Arm Error (deg)", getError());
        SmartDashboard.putBoolean("Is Arm At Set Point", isAtAngle());
        SmartDashboard.putBoolean("Arm Limit Switch", getLimitSwitchValue());
        SmartDashboard.putString("Arm Control Type", m_ArmControlType.toString());
        SmartDashboard.putString("Arm Control State", getArmControlState().toString());
        // SmartDashboard.putBoolean("left inverted", m_armMotorLeft.getInverted()); 
        // SmartDashboard.putBoolean("right inverted", m_armMotorRight.getInverted()); 
        // SmartDashboard.putNumber("left volt", m_armMotorLeft.get()); 
        // SmartDashboard.putNumber("right volt", m_armMotorRight.get()); 
    }
}
