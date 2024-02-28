package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Figures out which game pieces are near
public class FrontVision extends SubsystemBase{
     public static FrontVision instance  = new FrontVision();

    private PhotonCamera camera;
    private PhotonPipelineResult latestResult;

    private FrontVision() {
        this.camera = new PhotonCamera("frontPhotonCamera");
    }

    public void periodic() {
        this.latestResult = this.camera.getLatestResult();
        SmartDashboard.putNumber("Target Yaw",getNoteRotation());

    }

    public static FrontVision getInstance() {
        return instance;
    }

    public PhotonPipelineResult getLatestVisionResult() {
        return latestResult;
    }

    public double getNoteRotation() {
        if (this.latestResult == null || this.latestResult.getBestTarget() == null) return 0; 
        return latestResult.getBestTarget().getYaw();
    }

}
