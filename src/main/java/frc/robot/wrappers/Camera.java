package frc.robot.wrappers;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Camera {
    public PhotonCamera camera;

    public Camera(NetworkTableInstance nt) {
        camera = new PhotonCamera("OV5647");
    }

    public PhotonPipelineResult getResults() {
        return camera.getLatestResult();
    }

    public boolean hasTargets() {
        return getResults().hasTargets();
    }

    public Transform3d getTarget3D() {
        var target = camera.getLatestResult().getBestTarget();
        if(target == null) return null;
        return target.getBestCameraToTarget();
    }
    public Transform2d getTarget2D() {
        var target = getTarget3D();
        if(target == null) return null;
        return new Transform2d(new Translation2d(target.getX(), target.getY()), 
                                Rotation2d.fromRadians(target.getRotation().getZ()));
    }
    public Transform3d getRobot3D() {
        return getTarget3D().inverse();
    }
}
