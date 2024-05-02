package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// Figures out which game pieces are near
public class FrontVision extends SubsystemBase {
     public static FrontVision instance = new FrontVision();

    private PhotonCamera camera;
    private PhotonPipelineResult latestResult;
    private PhotonPipelineResult lastSuccessfulResult; 

    private FrontVision() {
        this.camera = new PhotonCamera("frontPhotonCamera");
    }

    public void periodic() {
        SmartDashboard.putBoolean("Front Camera is Connected", this.camera.isConnected()); 

        if (!this.camera.isConnected()) return; 

        try {
            this.latestResult = this.camera.getLatestResult();
        } catch (Exception err) {
            return;
        }

        if (this.latestResult == null) return; 

        if (this.latestResult.hasTargets()) lastSuccessfulResult = latestResult; 

        
        SmartDashboard.putNumber("Target Yaw",getNoteYaw());
    }

    public static FrontVision getInstance() {
        return instance;
    }

    public PhotonPipelineResult getLatestVisionResult() {
        return latestResult;
    }

    public PhotonPipelineResult getLastSuccessfulResult() {
        return this.lastSuccessfulResult;
    }

    public double getNoteYaw() {
        if (this.latestResult == null || !this.latestResult.hasTargets()) return 0; 
        return -latestResult.getBestTarget().getYaw();
    }

    public double getNotePitch() {
        if (this.latestResult == null || !this.latestResult.hasTargets()) return 0; 
        return latestResult.getBestTarget().getPitch();
    }

    public double getCurrentNoteForwardDistance() {
        return getNoteForwardDistance(Math.toRadians(getNoteYaw())); 
    }

    public double getCurrentNoteHorizontalDistance() {
        return getNoteHorizontalDistance(Math.toRadians(getNotePitch()), Math.toRadians(getNoteYaw())); 
    }

    public Transform2d getNotePose() {
        return new Transform2d(getCurrentNoteForwardDistance(), getCurrentNoteHorizontalDistance(), Rotation2d.fromDegrees(getNoteYaw())); 
    }

    public double getNoteForwardDistance(double pitch) {
        double pitchFromHorizontal = Math.abs(pitch + Constants.Vision.kFrontPitchDegrees);
        
        // law of sines to determine third distance
        double a = Constants.Vision.kCameraHeightMeters / Math.sin(pitchFromHorizontal); 

        return Math.sqrt(a * a - Math.pow(Constants.Vision.kCameraHeightMeters, 2)); 
    }

    public double getNoteHorizontalDistance(double pitch, double yaw) {
        double alpha = Math.PI / 2 - yaw; 
        double forwardDist = getNoteForwardDistance(pitch); 

        return forwardDist * Math.sin(yaw) / Math.sin(alpha); 
    }

}
