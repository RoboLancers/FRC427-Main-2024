package frc.robot.subsystems.drivetrain;

import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.SwerveModule.DriveState;
import frc.robot.util.ChassisState;
import frc.robot.util.SwerveUtils;

public class Drivetrain extends SubsystemBase {

  private static Drivetrain instance = new Drivetrain();

    public static Drivetrain getInstance() {
        return instance; 
    }

  // set up the four swerve modules  
  private SwerveModule frontLeft = new SwerveModule(Constants.DrivetrainConstants.frontLeft); 
  private SwerveModule frontRight = new SwerveModule(Constants.DrivetrainConstants.frontRight); 
  private SwerveModule backLeft = new SwerveModule(Constants.DrivetrainConstants.backLeft); 
  private SwerveModule backRight = new SwerveModule(Constants.DrivetrainConstants.backRight); 

  // initialize swerve position estimator
  private SwerveDrivePoseEstimator odometry; 
  private double m_Rotation;

  // initialize the gyro on the robot
  public AHRS gyro = new AHRS(SPI.Port.kMXP);

  // represents the current drive state of the robot
  private DriveState driveState = DriveState.CLOSED_LOOP; 

  private PIDController rotationController = new PIDController(
      Constants.DrivetrainConstants.kTurn_P,
      Constants.DrivetrainConstants.kTurn_I,
      Constants.DrivetrainConstants.kTurn_D
  ); 

  private Field2d m_odometryField = new Field2d(); 
  private Field2d m_visionField = new Field2d(); 

  private Drivetrain() {

    this.rotationController.enableContinuousInput(-180, 180); 

    // zero yaw when drivetrain first starts up
    this.gyro.zeroYaw();

    // create the pose estimator
    this.odometry = new SwerveDrivePoseEstimator(Constants.DrivetrainConstants.kDriveKinematics, getGyroRotation(), getPositions(), new Pose2d()); 
    
  }

  @Override
  public void periodic() {
    m_Rotation += rotationPerSecond*0.02;
    // update the odometry with the newest rotations, positions of the swerve modules
    SmartDashboard.putNumber("drive yaw", gyro.getYaw());
    SmartDashboard.putNumber("drive x", odometry.getEstimatedPosition().getX());
    SmartDashboard.putNumber("drive y", odometry.getEstimatedPosition().getY());
    SmartDashboard.putNumber("drive omega", odometry.getEstimatedPosition().getRotation().getDegrees());
    
    this.odometry.update(Rotation2d.fromRadians(m_Rotation), getPositions());

    m_odometryField.setRobotPose(getPose());

    SmartDashboard.putData("Robot Odometry Field", m_odometryField);
    SmartDashboard.putData("Robot Vision Field", m_visionField);
    doSendables();

    frontLeft.updateSim();
    frontRight.updateSim();
    backLeft.updateSim();
    backRight.updateSim();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void doSendables() {
    frontLeft.doSendables();
    frontRight.doSendables();
    backLeft.doSendables();
    backRight.doSendables();
  }

  
  /**
   * 
   * Drive the robot with a forward, sidewards, rotation speed
   * 
   * @param xMetersPerSecond the speed to drive forward with in meters per second
   * @param yMetersPerSecond the speed to drive sidewards with in meters per second
   * @param rotationRadPerSecond the speed to rotate at in radians per second
   * 
   */
  public void swerveDrive(double xMetersPerSecond, double yMetersPerSecond, double rotationRadPerSecond) {
    swerveDrive(new ChassisSpeeds(
      xMetersPerSecond, 
      yMetersPerSecond, 
      rotationRadPerSecond
    ));
  }

  private double rotationPerSecond = 0; 

  public void swerveDrive(ChassisSpeeds speeds) {
    Optional<Alliance> opt = DriverStation.getAlliance(); 
    if (!opt.isPresent()) return; 
    Alliance alliance = opt.get(); 
    ChassisSpeeds robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(
      speeds, 
      alliance == Alliance.Blue ? getRotation() : getRotation().plus(Rotation2d.fromDegrees(180))
    ); 

    // correct for drift in the chassis
    ChassisSpeeds correctedSpeeds = SwerveUtils.correctInputWithRotation(robotRelative); 

    // calculate module states from the target speeds
    SwerveModuleState[] states = Constants.DrivetrainConstants.kDriveKinematics.toSwerveModuleStates(correctedSpeeds); 

    // ensure all speeds are reachable by the wheel
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.DrivetrainConstants.kMaxAttainableModuleSpeedMetersPerSecond);

    rotationPerSecond = correctedSpeeds.omegaRadiansPerSecond; 

    swerveDrive(states);
  }

  public void swerveDriveWithoutCompensation(ChassisSpeeds speeds) {
    SwerveModuleState[] states = Constants.DrivetrainConstants.kDriveKinematics.toSwerveModuleStates(speeds); 
    // ensure all speeds are reachable by the wheel
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.DrivetrainConstants.kMaxAttainableModuleSpeedMetersPerSecond);
    
    rotationPerSecond = speeds.omegaRadiansPerSecond;

    swerveDrive(states);
  }


  public void swerveDriveRobotCentric(ChassisSpeeds speeds) {
    // correct for drift in the chassis
    ChassisSpeeds correctedSpeeds = SwerveUtils.correctInputWithRotation(speeds); 

    // calculate module states from the target speeds
    SwerveModuleState[] states = Constants.DrivetrainConstants.kDriveKinematics.toSwerveModuleStates(correctedSpeeds); 

    // ensure all speeds are reachable by the wheel
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.DrivetrainConstants.kMaxAttainableModuleSpeedMetersPerSecond);

    swerveDrive(states);
  }
  private double lastTurnedTheta = 0; 

  public void resetLastTurnedTheta() {
    lastTurnedTheta = this.getRotation().getDegrees();  
  }

  public void swerveDriveFieldRel(double xMetersPerSecond, double yMetersPerSecond, double thetaDegrees, boolean turn) {
    
   // if (turn) lastTurnedTheta = thetaDegrees; 
    
    // always make an effort to rotate to the last angle we commanded it to

    // to commit to going to our angle even after stopping pressing
   // double rotSpeed = rotationController.calculate(this.getYaw(), lastTurnedTheta); 

    // or to not commit to the angle
     if (turn
      // || gyro.getRate() > 0.25
      ) lastTurnedTheta = this.getRotation().getDegrees(); 
     double rotSpeed = rotationController.calculate(getRotation().getDegrees(), turn ? thetaDegrees : lastTurnedTheta); 
     

    rotSpeed = MathUtil.clamp(rotSpeed, -Constants.DrivetrainConstants.kMaxRotationRadPerSecond, Constants.DrivetrainConstants.kMaxRotationRadPerSecond); 

    swerveDrive(xMetersPerSecond, yMetersPerSecond, rotSpeed);
  }

  public void swerveDriveFieldRel(ChassisState state) {
    swerveDriveFieldRel(state.vxMetersPerSecond, state.vyMetersPerSecond, Math.toDegrees(state.omegaRadians), state.turn);
  }

  // command the swerve modules to the intended states
  public void swerveDrive(
    SwerveModuleState[] states) {
      this.frontLeft.updateState(states[0], driveState);
      this.frontRight.updateState(states[1], driveState);
      this.backLeft.updateState(states[2], driveState);
      this.backRight.updateState(states[3], driveState);
  }

  // returns the positions of all the swerve modules
  public SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPosition(), 
      frontRight.getPosition(), 
      backLeft.getPosition(), 
      backRight.getPosition()
    }; 
  }
  
  // returns the speeds and angles of all the swerve modules
  public SwerveModuleState[] getStates() {
    return new SwerveModuleState[] {
      frontLeft.getCurrentState(), 
      frontRight.getCurrentState(), 
      backLeft.getCurrentState(), 
      backRight.getCurrentState()
    }; 
  }



  // returns the speed of the robot 
  public ChassisSpeeds getChassisSpeeds() {
    return Constants.DrivetrainConstants.kDriveKinematics.toChassisSpeeds(getStates()); 
  }

  // returns the current odometry pose of the robot
  public Pose2d getPose() {
    return odometry.getEstimatedPosition(); 
  }

  // returns the direction the robot is facing in degrees from -180 to 180 degrees.
  public double getHeading() {
      return gyro.getRotation2d().getDegrees();
  }


  public Rotation2d getRotation() {
    return odometry.getEstimatedPosition().getRotation(); 
  }

  public void setRotation(Rotation2d rotation) {
    resetOdometry(new Pose2d(getPose().getX(), getPose().getY(), rotation));
  }

  // zeros the current heading of the robot
  public void zeroHeading() {
    setRotation(new Rotation2d());
  }

  // gets the raw heading of the robot
  public Rotation2d getGyroRotation() {
      return Rotation2d.fromRadians(m_Rotation);
  }

  // Returns the rate at which the robot is turning in degrees per second.
  public double getTurnRate() {
      return -gyro.getRate();
  }

  // reset the current pose of the robot to a set pose
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getGyroRotation(), getPositions(), pose);
  }

  /**
   * set the drive state of the robot
   * @see DriveState
   */ 
  public void setDriveState(DriveState state) {
    this.driveState = state; 
  }

  public void addVisionPoseEstimate(Pose3d pose3d, double targetDistance, double timestamp, Matrix<N3, N1> stdDevs) {
    odometry.addVisionMeasurement(pose3d.toPose2d(), timestamp, stdDevs);

    m_visionField.setRobotPose(pose3d.toPose2d());
  }
}
