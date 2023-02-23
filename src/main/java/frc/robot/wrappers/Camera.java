package frc.robot.wrappers;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.util.Statics;

public class Camera {
    public PhotonCamera camera;

    private double[] tagConfidence = new double[Constants.aprilTags.length];
    private boolean[] tagVisible = new boolean[Constants.aprilTags.length];
    private Pose3d[] robotPoses = new Pose3d[Constants.aprilTags.length];

    private double CONFIDENCE_LINEAR = 1.0; // visble target raw confidence gain / second
    private double CONFIDENCE_FORGET = 1.0; // lost target raw confidence loss / second
    private double CONFIDENCE_DISTANCE = 200; // confidence loss over area of target
    private double CONFIDENCE_AMBIGUITY = 0.1; // confidence loss for photon Ambiguity
    private double CONFIDENCE_ALTITUDE = 10.0; // confidence lost per meter the robot is above/below the ground

    private Timer timer = new Timer();

    private double lastT = 0.0;
    private double dt = 1.0;

    private Pose2d estimatedRobotPose = new Pose2d();
    private double poseConfidence = 0.0;

    public Camera(NetworkTableInstance nt) {
        camera = new PhotonCamera("OV5647");
        timer.start();
    }

    public PhotonPipelineResult getResults() {
        return camera.getLatestResult();
    }

    public boolean hasTargets() {
        return getResults().hasTargets();
    }

    public Transform3d getTarget3D() {
        PhotonTrackedTarget target = camera.getLatestResult().getBestTarget();
        if(target == null) return null;
        return target.getBestCameraToTarget();
    }
    public Transform2d getTarget2D() {
        Transform3d target = getTarget3D();
        if(target == null) return null;
        return new Transform2d(new Translation2d(target.getX(), target.getY()), 
                                Rotation2d.fromRadians(target.getRotation().getZ()));
    }
    public Transform3d getTagToRobot(){
        return getTarget3D().inverse();
    }
    private Pose3d[] scanTags3d() {
        List<PhotonTrackedTarget> targets = camera.getLatestResult().targets;
        Pose3d[] robotPositions = new Pose3d[targets.size()];

        for (int i = 0 ; i < robotPositions.length; i++) {
            PhotonTrackedTarget target = targets.get(i);

            if(target == null) return null;
            Transform3d cameraToTag = target.getBestCameraToTarget();
            Transform3d robotToTag = cameraToTag.plus(
                new Transform3d(Constants.cameraPose.getTranslation(), Constants.cameraPose.getRotation()).inverse()
                );

            int tagID = target.getFiducialId();
            Transform3d tagToRobot = robotToTag.inverse();
            Pose3d tagFieldPosition = Constants.aprilTags[tagID-1].pose;
            Pose3d robotPosition = tagFieldPosition.transformBy(tagToRobot);

            robotPositions[i] = robotPosition;
            robotPoses[tagID-1] = robotPosition;
            tagVisible[tagID-1] = true;
            addConfidence(target, robotPosition);
                
        }

        return robotPositions;
        
    }
    private void addConfidence(PhotonTrackedTarget target, Pose3d estimatedRobotPose){
        int confidenceIndex = target.getFiducialId()-1;

        double confidence = CONFIDENCE_LINEAR*dt;
        confidence -= Math.abs(estimatedRobotPose.getZ())*CONFIDENCE_ALTITUDE;

        // V Better safe than sorry!
        if(target.getArea() != 0) confidence -= CONFIDENCE_DISTANCE/target.getArea();

        confidence -= target.getPoseAmbiguity()*CONFIDENCE_AMBIGUITY;

        tagConfidence[confidenceIndex] = Math.min(confidenceIndex+confidence, 1.0);
    }

    public Pose3d calcWeightedPose(){
        Pose3d totalPose = new Pose3d();
        double[] weights = new double[tagConfidence.length];
        for (int i = 0; i < weights.length; i++) { weights[i] = tagConfidence[i]*(tagVisible[i] ? 1 : 0); }
        
        weights = Statics.normalizeSum(weights);
        
        for(int index = 0; index < tagConfidence.length; index++){
            totalPose = Statics.sumPoses(totalPose, robotPoses[index].times(weights[index]));
        }
        return totalPose;
    }

    public void update(){

        for (int i = 0; i < tagVisible.length; i++) { tagVisible[i] = false; }
        scanTags3d();

        Pose3d robotPose3d = calcWeightedPose();
        poseConfidence = 0.0;

        for(int index = 0; index < tagConfidence.length; index++){
            poseConfidence = Math.max(poseConfidence, tagConfidence[index]);
            tagConfidence[index] -= CONFIDENCE_FORGET*dt;
        }

        estimatedRobotPose = new Pose2d(new Translation2d(robotPose3d.getX(), robotPose3d.getY()),
            new Rotation2d(robotPose3d.getRotation().getZ()));
        
        dt = timer.get()-lastT;
        lastT = timer.get();
    }

    public Pose2d getRobotPose2d(){
        return estimatedRobotPose;
    }
    public double getPoseConfidence(){
        return poseConfidence;
    }
    
}
