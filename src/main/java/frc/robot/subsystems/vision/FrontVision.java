package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

        
        SmartDashboard.putNumber("Target Yaw",getNoteRotation());
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

    public double getNoteRotation() {
        if (this.latestResult == null || !this.latestResult.hasTargets()) return 0; 
        return latestResult.getBestTarget().getYaw();
    }

}
