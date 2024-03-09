package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.MotorSim;
import frc.robot.util.MotorSim.Mode;

// represents a single swerve pod on the robot
public class SwerveModule {

    // current target state (speed, angle) of the swerve pod
    private SwerveModuleState targetState;
    
    // the turn and drive motors of the swerve pod
    private MotorSim turnMotor; 
    private MotorSim driveMotor;
    
    // PID controllers for each respective motor
    public SwerveTurnPIDController turnPIDController; 
    private SparkPIDController drivePIDController; 

    // encoder for the angle of the wheel relative to 0 degrees (forward)
    //private CANcoder absoluteTurnEncoder;

    // encoder for the drive wheel
   // private RelativeEncoder driveMotor; 

    // feedforward values of the drive, not necessarily needed
    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
        Constants.DrivetrainConstants.ksVolts, 
        Constants.DrivetrainConstants.kvVoltSecondsPerMeter, 
        Constants.DrivetrainConstants.kaVoltSecondsSquaredPerMeter
    ); 

    private String name; 

    private DriveState driveType = DriveState.CLOSED_LOOP; 

    private double currentDrivePos = 0; 
    private double lastDrivePos = 0; 

    /**
     * 
     * @param config the config of the corresponding swerve module
     */
    public SwerveModule(SwerveModuleConfig config) {

        int kTurn = config.getRotateId(); 
        int kDrive = config.getDriveId(); 
        int kTurnEncoder = config.getEncoderId();
        double kOffset = config.getAbsoluteEncoderOffset(); 

        this.name = config.getName(); 

        this.turnMotor = new MotorSim(kTurn, MotorType.kBrushless, Mode.MANUAL); 
        this.driveMotor = new MotorSim(kDrive, MotorType.kBrushless, Mode.MANUAL); 

        // this.absoluteTurnEncoder = new CANcoder(kTurnEncoder); 
       // this.driveMotor = this.driveMotor.getEncoder(); 

      //  this.turnPIDController = new SwerveTurnPIDController(absoluteTurnEncoder, 0, 0, 0); 
       // this.drivePIDController = this.driveMotor.getPIDController(); 

        // this.currentDrivePos = this.driveEncoder.getPosition(); 
        // this.lastDrivePos = this.driveEncoder.getPosition();

        configureMotors(config.getDriveInverted(), config.getRotateInverted());
        configureEncoders(config.getAbsoluteEncoderDirection(), kOffset);
        configurePIDControllers();

        // in case of brownout
        // this.turnMotor.burnFlash(); 
        // this.driveMotor.burnFlash(); 
    }

    // Sets current limits, idle modes, etc. for each motor for maximum performance
    private void configureMotors(boolean driveInverted, boolean rotateInverted) {
        this.driveMotor.setSmartCurrentLimit(Constants.DrivetrainConstants.kDriveCurrentLimit); 
        // this.driveMotor.setIdleMode(IdleMode.kBrake); 
        // this.driveMotor.enableVoltageCompensation(12); 
        // this.driveMotor.setClosedLoopRampRate(Constants.DrivetrainConstants.kDriveRampRate);
        // this.driveMotor.setOpenLoopRampRate(Constants.DrivetrainConstants.kDriveRampRate);
        this.driveMotor.setInverted(driveInverted);
         

        this.turnMotor.setSmartCurrentLimit(Constants.DrivetrainConstants.kTurnCurrentLimit); 
        // this.turnMotor.setIdleMode(IdleMode.kBrake); 
        // this.turnMotor.enableVoltageCompensation(12); 
        // this.turnMotor.setClosedLoopRampRate(Constants.DrivetrainConstants.kTurnRampRate);
        // this.turnMotor.setOpenLoopRampRate(Constants.DrivetrainConstants.kTurnRampRate);
        this.turnMotor.setInverted(rotateInverted);
    }

    public void doSendables() {
        SmartDashboard.putNumber(name + " Turn Vel (arb)", this.turnMotor.getVelocity());
        SmartDashboard.putNumber(name + " Drive Vel (m/s)", this.driveMotor.getVelocity());

        if (this.getReferenceState() != null) SmartDashboard.putNumber(name + "Commanded Drive Vel (m/s)", this.getReferenceState().speedMetersPerSecond); 

        SmartDashboard.putNumber(name + " Abs Turn Angle (degrees)", this.getAngle().getDegrees());
    }

    // sets the conversion factors for the drive encoder based on gear ratios
    private void configureEncoders(SensorDirectionValue direction, double kAbsoluteOffset) {
        this.driveMotor.setPositionConversionFactor(Constants.DrivetrainConstants.kMetersPerRot);
        this.driveMotor.setVelocityConversionFactor(Constants.DrivetrainConstants.kMetersPerSecondPerRPM); 

        final MagnetSensorConfigs config = new MagnetSensorConfigs(); 
        config.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf; 
        config.MagnetOffset = kAbsoluteOffset; 
        config.SensorDirection = direction; 

        //this.absoluteTurnEncoder.getConfigurator().apply(config); 
    }

    // sets the PID constants for turn & drive
    private void configurePIDControllers() {
        setTurnPID(
            Constants.DrivetrainConstants.kModuleTurn_P, 
            Constants.DrivetrainConstants.kModuleTurn_I, 
            Constants.DrivetrainConstants.kModuleTurn_D
            );
        
        setDrivePID(Constants.DrivetrainConstants.kModuleDrive_P, Constants.DrivetrainConstants.kModuleDrive_I, Constants.DrivetrainConstants.kModuleDrive_D, Constants.DrivetrainConstants.kModuleDrive_FF);
    }

    // sets the PID constants for the turn motor
    public void setTurnPID(double p, double i, double d) {
      //  this.turnPIDController.setPID(p, i, d);
    }

    // sets the PID constants for the drive motor
    public void setDrivePID(double p, double i, double d, double ff) {
        // this.drivePIDController.setP(p); 
        // this.drivePIDController.setI(i); 
        // this.drivePIDController.setD(d); 
        // this.drivePIDController.setFF(ff); 
    }

    /**
     * 
     * @param state the target speed & angle for the module to go to, in the form of a {@link SwerveModuleState}
     * @param driveType the type of drive to operate with
     */
    public void updateState(SwerveModuleState state, DriveState driveType) {
        // SmartDashboard.putNumber("module " + absoluteTurnEncoder.getDeviceID() + " desired speed", state.speedMetersPerSecond); 
        // SmartDashboard.putNumber("module " + absoluteTurnEncoder.getDeviceID() + " actual speed", this.driveMotor.getVelocity()); 
        // SmartDashboard.putNumber("module " + absoluteTurnEncoder.getDeviceID() + " diff speed", state.speedMetersPerSecond - this.driveMotor.getVelocity()); 

        this.targetState = state; 
        this.driveType = driveType;

    }

    public void commandState() {

        // double encPos = driveEncoder.getPosition();

        // if (Math.abs(encPos - this.lastDrivePos) >= 3
        //  || (Math.abs(encPos) <= 0.1 && Math.abs(this.lastDrivePos) > 0.1)) {
        //     // do nothing
        // } else {
        //     this.currentDrivePos += encPos - this.lastDrivePos;
        // }

        // this.lastDrivePos = encPos;

        if (this.targetState == null) return; 
        // SmartDashboard.putNumber("module " + name + " position", this.driveEncoder.getPosition()); 
        // SmartDashboard.putNumber("module " + name + " desired speed", this.targetState.speedMetersPerSecond); 
        // SmartDashboard.putNumber("module " + name + " actual speed", this.driveEncoder.getVelocity()); 
        // SmartDashboard.putNumber("module " + name + " diff speed", this.targetState.speedMetersPerSecond - this.driveEncoder.getVelocity()); 

        // optimize angles so the wheels only have to turn 90 degrees to reach their setpoint at any given time
        SwerveModuleState optimizedState = SwerveModuleState.optimize(this.targetState, getAngle()); 

        if (driveType == DriveState.OPEN_LOOP) updateOpenLoopDriveState(optimizedState.speedMetersPerSecond); 
         else updateClosedLoopDriveState(optimizedState.speedMetersPerSecond);

         updateTurnState(optimizedState.angle);
    }

    // handle open loop drive; calculate # output based on theoretical motor maximums
    private void updateOpenLoopDriveState(double speed) {
        double percent = MathUtil.clamp(speed / Constants.DrivetrainConstants.kMaxAttainableModuleSpeedMetersPerSecond, -1, 1); 
        driveMotor.set(percent);
    }
    
    // handle closed loop drive; use SparkMAX's PID controller to go to the specified speed
    private void updateClosedLoopDriveState(double speed) {
        // drivePIDController.setReference(speed, ControlType.kVelocity, 0, driveFeedforward.calculate(speed)); 
        driveMotor.set(speed);
    }

    /**
     * Turns the pod to the desired angle 
     * @param turn the desired angle to turn to
     */
    private void updateTurnState(Rotation2d turn) {
        // turnPIDController.setSetpoint(turn.getDegrees()); 
        // turnMotor.set(turnPIDController.calculate());
        turnMotor.setCurrentPosition(turn.getDegrees());
    }

    // current angle of the swerve pod
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(this.turnMotor.getPosition()); 
    }

    // current state (velocity & angle) of the swerve pod
    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(this.driveMotor.getVelocity(), getAngle()); 
    }

    // current position (position & angle) of the swerve pod
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(this.driveMotor.getPosition(), getAngle()); 
    }

    // target state (velocity & angle) of the swerve pod
    public SwerveModuleState getReferenceState() {
        return this.targetState; 
    }

    public void updateSim() {
        turnMotor.update(0.02);
        driveMotor.update(0.02);
    }

    public static enum DriveState {
        OPEN_LOOP, // drive the motor purely with calculations of how fast it should do; does not take into account resistance
        CLOSED_LOOP // discretely controls the motor's speed based on past motor speed data using PID
    }
}
