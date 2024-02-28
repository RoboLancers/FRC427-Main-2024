package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MotorSim;
import frc.robot.util.MotorSim.Mode;

 /*
  * Notes about how Intake works
  - theres 2 main parts, The SHOOTER (intakMoterShootTOP/BOTTOM) and the SUCKER (outtakeMotorSuck)
  - the shooter has 2 motors Top and Bottom (teh bottom one follows the top) and the sucker has only one 
  - (the naming conventions are a little swapped thats my fault yo...)
  - so the sucker, SUCKS the rings from the ground and puts them into the shooter.
  - Then the shooter takes the rings from the sucker and shoots them into whatever it needs
  */

public class Intake extends SubsystemBase {

    private static Intake instance = new Intake();

    public static Intake getInstance() {
        return instance; 
    }

    // desire speed
    private double m_desireSpeed;
    
    // establishes the motors for shooter and sucker. Also establishes the beambreak.
    MotorSim m_intakeMotorShootTop = new MotorSim(Constants.IntakeConstants.kIntakeMotorShootTopId, MotorType.kBrushless, Mode.MANUAL);
    //RelativeEncoder m_intakeEncoderShootTop = m_intakeMotorShootTop.getEncoder();

    MotorSim m_intakeMotorShootBottom = new MotorSim(Constants.IntakeConstants.kIntakeMotorShootBottomId, MotorType.kBrushless, Mode.MANUAL);
   // RelativeEncoder m_intakeEncoderShootBottom = m_intakeMotorShootBottom.getEncoder();

    MotorSim m_outtakeMotorSuck = new MotorSim(Constants.IntakeConstants.kOuttakeMotorSuckId, MotorType.kBrushless, Mode.MANUAL);
    //RelativeEncoder m_outtakeEncoderSuck = m_outtakeMotorSuck.getEncoder();

    SparkPIDController m_intakePidController = m_intakeMotorShootTop.getPIDController();


    // beambreak checks for if theres a note in the intake
    DigitalInput m_BeamBreak = new DigitalInput(Constants.IntakeConstants.kBeamBreakId); 

    
    private Intake() {
        setupMotors();
        setupControllers();

    }
    public void setupMotors() {
        // sets limits for all the motors and has the bottom shoot motor, follow the top
        m_intakeMotorShootTop.setInverted(Constants.IntakeConstants.kShootTopIntakeInverted);
        m_intakeMotorShootTop.setSmartCurrentLimit(Constants.IntakeConstants.kShootTopMotorlimit);
        m_intakeMotorShootTop.setVelocityConversionFactor(Constants.IntakeConstants.kShootVelocityConversionFactor); 

        m_intakeMotorShootBottom.setInverted(Constants.IntakeConstants.kShootBottomIntakeInverted);
        m_intakeMotorShootBottom.setSmartCurrentLimit(Constants.IntakeConstants.kShootBottomMotorlimit);
        m_intakeMotorShootTop.setVelocityConversionFactor(Constants.IntakeConstants.kShootVelocityConversionFactor); 
        m_intakeMotorShootBottom.follow(m_intakeMotorShootTop);

        m_outtakeMotorSuck.setInverted(Constants.IntakeConstants.kSuckOuttakeInverted);
        m_outtakeMotorSuck.setSmartCurrentLimit(Constants.IntakeConstants.kSuckOuttakeMotorLimit);
        m_outtakeMotorSuck.setVelocityConversionFactor(Constants.IntakeConstants.kIntakeVelocityConversionFactor); 
    }

    public void setupControllers() {
        m_intakePidController.setP(Constants.IntakeConstants.kP);
        m_intakePidController.setI(Constants.IntakeConstants.kI);
        m_intakePidController.setD(Constants.IntakeConstants.kD);
        m_intakePidController.setFF(Constants.IntakeConstants.kFF);
    }

    public void periodic() {
        m_intakeMotorShootBottom.update(0.02);
        m_intakeMotorShootTop.update(0.02);
        m_outtakeMotorSuck.update(0.02);
        // code inside here will run repeatedly while the robot is on
        m_intakePidController.setReference(m_desireSpeed, CANSparkBase.ControlType.kVelocity);
        doSendables();
    }
    //so intaking the ring is sucking it
    public void intakeRing(double speed) {
        m_outtakeMotorSuck.set(speed);
    }
    //and outtaking the ring is shooting it
    public void outtakeRing(double speed) {
        // m_intakeMotorShootTop.set(speed);
        this.m_desireSpeed = speed;
        
    }
    //beambreak is a scanner that checks if a ring is inside the whole intake
    public boolean beamBreakHit() { 
        return !m_BeamBreak.get();
    }
    public void stopSuck() {
        m_outtakeMotorSuck.set(0); 
    }
    
    public void stopShoot() {
        // m_intakeMotorShootTop.set(0);
        outtakeRing(0);
    }

    public boolean atDesiredShootSpeed() {
        return (Math.abs(m_intakeEncoderShootTop.getVelocity() - m_desireSpeed) <= Constants.IntakeConstants.kTolerance); 
    }

     public void doSendables() { 
        SmartDashboard.putNumber("Suck Speed (m/s)", m_outtakeMotorSuck.getVelocity());
        SmartDashboard.putNumber("Shoot Top Speed (m/s)", m_intakeMotorShootTop.getVelocity());
        SmartDashboard.putNumber("Shoot Bottom Speed (m/s)", m_intakeMotorShootBottom.getVelocity());
        SmartDashboard.putBoolean("Beam Break Hit (t/f)", beamBreakHit());
    }
}