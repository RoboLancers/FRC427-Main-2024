package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.VecBuilder;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import edu.wpi.first.wpilibj.Timer;

public class Vision_old extends SubsystemBase {

    
    private static Vision_old instance = new Vision_old(Drivetrain.getInstance());

    public static Vision_old getInstance() {
        return instance; 
    }

    private static NetworkTable limelightNT = NetworkTableInstance.getDefault().getTable("limelight");
    private double limelightX;
    private double limelightY;
    private double limelightZ;
    private double limelightRoll;
    private double limelightPitch;
    private double limelightYaw;
    private double limelightTotalLatency;
    private double limelightTargetArea;
    private double limelightTargetX;
    private double limelightTargetY;
    private Drivetrain drivetrain;
    private double[] botPoseValues;

    // Creates a new limelight object
    private Vision_old(Drivetrain drivetrain) {
        limelightNT = NetworkTableInstance.getDefault().getTable("limelight");
        this.drivetrain = drivetrain;
    }

    // Updates the values for the robot position and camera position
    public void periodic() {
        // Gets the values scored in the table
        NetworkTableEntry botPose = limelightNT.getEntry("botpose_wpiblue");

        // Reads values periodically
        botPoseValues = botPose.getDoubleArray(new double[7]); // Make sure these are the right indices

        // Converts these values into variables
        limelightX = botPoseValues[0];
        limelightY = botPoseValues[1];
        limelightZ = botPoseValues[2];
        limelightRoll = botPoseValues[3];
        limelightPitch = botPoseValues[4];
        limelightYaw = botPoseValues[5];
        limelightTotalLatency = botPoseValues[6];
        limelightTargetArea = limelightNT.getEntry("tA").getDouble(0);
        limelightTargetX = limelightNT.getEntry("tX").getDouble(0);
        limelightTargetY = limelightNT.getEntry("tY").getDouble(0);

        // Posts to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", limelightX);
        SmartDashboard.putNumber("LimelightY", limelightY);
        SmartDashboard.putNumber("LimelightZ", limelightZ);
        SmartDashboard.putNumber("LimelightRoll", limelightRoll);
        SmartDashboard.putNumber("LimelightPitch", limelightPitch);
        SmartDashboard.putNumber("LimelightYaw", limelightYaw);
        SmartDashboard.putNumber("LimelightTotalLatency", limelightTotalLatency);
        SmartDashboard.putNumber("LimelightTargetArea", limelightTargetArea);
        SmartDashboard.putNumber("LimelightTargetX", limelightTargetX);
        SmartDashboard.putNumber("LimelightTargetY", limelightTargetY);
        // SmartDashboard.putNumber("LimelightDistanceToTarget", getDistanceToAprilTag());
        SmartDashboard.putNumber("LimelightNearestAprilTag", getClosestAprilTagID());
        // if (getAprilTagPos(getClosestAprilTagID()) != null) SmartDashboard.putNumber("LimelightNearestAprilTagPositionX", getAprilTagPos(getClosestAprilTagID()).getX());
        // if (getAprilTagPos(getClosestAprilTagID()) != null) SmartDashboard.putNumber("LimelightNearestAprilTagPositionY", getAprilTagPos(getClosestAprilTagID()).getY());
        // if (getAprilTagPos(getClosestAprilTagID()) != null) SmartDashboard.putNumber("LimelightNearestAprilTagPositionZ", getAprilTagPos(getClosestAprilTagID()).getZ());

        if (getAprilTagPos(getClosestAprilTagID()) != null) addVisionFromDrivetrain();
    }

    // Calculates the stand deviation of the distance of the object
    private Matrix<N3, N1> calculateVisionStdDevs() {
        double distance = getDistanceToAprilTag();
        var translationStdDev = Constants.Vision.kTranslationStdDevCoefficient * Math.pow(distance, 2);
        var rotationStdDev = Constants.Vision.kRotationStdDevCoefficient * Math.pow(distance, 2);

        SmartDashboard.putNumber("translation stddev", translationStdDev); 
        SmartDashboard.putNumber("rotation stddev", rotationStdDev); 
        return VecBuilder.fill(translationStdDev, translationStdDev, rotationStdDev);
    }

    // Gets the id of the closest april tag
    private int getClosestAprilTagID() {
        return (int) limelightNT.getEntry("tid").getInteger(-1);
    }

    // Returns the pose3d of the closest april tag
    private Pose3d getAprilTagPos(int aprilTagID) {
        return Constants.Vision.kAprilTagFieldLayout.getTagPose(aprilTagID).orElse(null);
    }

    // Gets the distance to the closest april tag by finding the 3-dimensional norm
    private double getDistanceToAprilTag() {
        Pose3d aprilTagPose = getAprilTagPos(getClosestAprilTagID());
        if (aprilTagPose == null) return 1000;
        return getDistanceBetweenPose3d(aprilTagPose, getCurrentPose3d());
    }

    // Returns the distance between two pose3d objects in three dimensions
    private double getDistanceBetweenPose3d(Pose3d firstPose, Pose3d secondPose) {
        double xDifferenceSquared = Math.pow(secondPose.getX() - firstPose.getX(), 2);
        double yDifferenceSquared = Math.pow(secondPose.getY() - firstPose.getY(), 2);
        double zDifferenceSquared = Math.pow(secondPose.getZ() - firstPose.getZ(), 2);
        return Math.sqrt(xDifferenceSquared + yDifferenceSquared + zDifferenceSquared);
    }

     // Returns the distance between two pose3d objects in three dimensions
    private double getDistanceBetweenPose2d(Pose2d firstPose, Pose2d secondPose) {
        double xDifferenceSquared = Math.pow(secondPose.getX() - firstPose.getX(), 2);
        double yDifferenceSquared = Math.pow(secondPose.getY() - firstPose.getY(), 2);
        return Math.sqrt(xDifferenceSquared + yDifferenceSquared);
    }

    // Returns the rotation3d using values stored in the network table
    private Rotation3d getRotation3d() {
        return new Rotation3d(Math.toRadians(limelightRoll), Math.toRadians(limelightPitch), Math.toRadians(limelightYaw));
    }

    // Returns the rotation3d using values stored in the network table
    private Pose3d getCurrentPose3d() {
        return new Pose3d(limelightX, limelightY, limelightZ, getRotation3d());
    }

    // Adds vision measurements to the drivetrain if they are within the field
    public void addVisionFromDrivetrain() {
        if (!isPoseValid()) return; 
        drivetrain.addVisionPoseEstimate(getCurrentPose3d(), getDistanceToAprilTag(), Timer.getFPGATimestamp() - (limelightTotalLatency/1000.0), calculateVisionStdDevs());
    }

    public boolean isEstimateClose() {
        Pose2d currentPose = new Pose2d(limelightX, limelightY, Rotation2d.fromDegrees(0));
        return getDistanceBetweenPose2d(currentPose, drivetrain.getPose()) < Constants.Vision.kMaxAccuracyRange;
    }

    // Sees if the robot's position as given by the limelight is within the field
    public boolean isPoseValid() {
        boolean isInLowerXLimit = -2.5 <= limelightX;
        boolean isInUpperXLimit = limelightX <= Constants.Vision.kAprilTagFieldLayout.getFieldLength() + 2.5;
        boolean isInLowerYLimit = -2.5 <= limelightY;
        boolean isInUpperYLimit = limelightY <= Constants.Vision.kAprilTagFieldLayout.getFieldWidth() + 2.5;
        boolean isInLowerZLimit = 0 <= limelightZ;
        boolean isInUpperZLimit = limelightZ <= Constants.Vision.limelightZHeight + 0.5;
        return isInLowerXLimit && isInUpperXLimit && isInLowerYLimit && isInUpperYLimit && isInLowerZLimit && isInUpperZLimit;
    }

    public double getYaw() {
        return this.limelightYaw;
    }
}