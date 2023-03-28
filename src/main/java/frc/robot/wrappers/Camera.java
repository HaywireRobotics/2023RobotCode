package frc.robot.wrappers;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.Statics;

public class Camera {
    public PhotonCamera camera;

    private double[] tagConfidence = new double[Constants.aprilTags.length];
    private boolean[] tagVisible = new boolean[Constants.aprilTags.length];
    private Pose3d[] robotPoses = new Pose3d[Constants.aprilTags.length];

    private double CONFIDENCE_LINEAR = 4.0; // visble target raw confidence gain / second
    private double CONFIDENCE_FORGET = 0.1; // lost target raw confidence loss / second
    private double CONFIDENCE_DISTANCE = 0.005; // confidence loss over area of target
    private double CONFIDENCE_AMBIGUITY = 0.1; // confidence loss for photon Ambiguity
    private double CONFIDENCE_ALTITUDE = 7.0; // confidence lost per meter the robot is above/below the ground
    private double CONFIDENCE_PAST = 0.1;

    private Pose3d cameraMountPose;

    private Timer timer = new Timer();

    private double lastT = 0.0;
    private double dt = 1.0;

    private Pose2d estimatedRobotPose = new Pose2d();
    private double poseConfidence = 0.0;

    
    private NetworkTable table;

    private DoublePublisher confidencePublisher;
    private IntegerPublisher numTagsPublisher;
    private DoubleArrayPublisher estimatedPosePublisher;

    public Camera(NetworkTableInstance nt, String name, Pose3d cameraMountPose) {
        camera = new PhotonCamera(name);
        this.cameraMountPose = cameraMountPose;
        timer.start();

        table = nt.getTable("AprilTagPipeline");
        confidencePublisher = table.getDoubleTopic("Total Confidence").publish();
        numTagsPublisher = table.getIntegerTopic("NumTags").publish();
        estimatedPosePublisher = table.getDoubleArrayTopic("Estimated Pose").publish();
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
            int tagID = target.getFiducialId();
            if (tagID > Constants.aprilTags.length || tagID < 1) continue;
            
            Transform3d cameraToTag = target.getBestCameraToTarget();
            Transform3d cameraToRobot = new Transform3d(cameraMountPose.getTranslation(), cameraMountPose.getRotation());//.inverse();
            // Transform3d robotToTag = cameraToRobot.plus(cameraToTag);

            // Transform3d tagToCamera = cameraToTag.inverse();
            Transform3d tagToRobot = cameraToTag.plus(cameraToRobot);
            Pose3d tagFieldPosition = Constants.aprilTags[tagID-1].pose;//new Pose3d(new Translation3d(), new Rotation3d(0, -Math.PI/2, 0)); //
            // Pose3d cameraPose = tagFieldPosition.transformBy(tagToCamera);
            Pose3d _robotPose = tagFieldPosition.transformBy(tagToRobot);
            Pose3d robotPose = new Pose3d(new Translation3d(_robotPose.getX(), _robotPose.getY(), -_robotPose.getZ()), _robotPose.getRotation());

            double[] poseArray = {cameraToTag.getZ(), cameraToTag.getX(), cameraToTag.getY()};
            if(poseArray[0] == 0 && poseArray[1] == 0 && poseArray[2] == 0){
                System.out.println(cameraToTag.getX()+", "+cameraToTag.getY()+" "+ cameraToTag.getZ());
            }
            // estimatedPosePublisher.set(poseArray, 3);
            // System.out.println(robotPose.getX()+", "+robotPose.getY()+" "+ robotPose.getZ());
            robotPositions[i] = robotPose;
            robotPoses[tagID-1] = robotPose;
            tagVisible[tagID-1] = true;
            addConfidence(target, robotPose);
                
        }
        
        numTagsPublisher.set(targets.size());

        return robotPositions;
        
    }
    private void addConfidence(PhotonTrackedTarget target, Pose3d estimatedRobotPose){
        int confidenceIndex = target.getFiducialId()-1;

        double confidence = 1;//CONFIDENCE_LINEAR*dt;  SmartDashboard.putNumber("Camera: Linear", CONFIDENCE_LINEAR*dt);
        confidence -= Math.abs(estimatedRobotPose.getZ())*CONFIDENCE_ALTITUDE; SmartDashboard.putNumber("Camera: Altitude", Math.abs(estimatedRobotPose.getZ())*CONFIDENCE_ALTITUDE);

        // V Better safe than sorry!
        if(target.getArea() != 0) confidence -= CONFIDENCE_DISTANCE/target.getArea(); SmartDashboard.putNumber("Camera: Distance", CONFIDENCE_DISTANCE/target.getArea());

        confidence -= target.getPoseAmbiguity()*CONFIDENCE_AMBIGUITY; SmartDashboard.putNumber("Camera: Ambiguity", target.getPoseAmbiguity()*CONFIDENCE_AMBIGUITY);

        SmartDashboard.putNumber("Camera: Total Confidence", confidence);

        tagConfidence[confidenceIndex] = Math.max(Math.min(tagConfidence[confidenceIndex]*CONFIDENCE_PAST+confidence, 1.0), 0.0);
    }

    public Pose3d calcWeightedPose(){
        Pose3d totalPose = new Pose3d();
        double[] weights = new double[tagConfidence.length];
        for (int i = 0; i < weights.length; i++) { weights[i] = tagConfidence[i]*(tagVisible[i] ? 1 : 0); }
        
        weights = Statics.normalizeSum(weights);
        
        for(int index = 0; index < tagConfidence.length; index++){
            if(robotPoses[index] != null && tagConfidence[index] != 0){
            totalPose = Statics.sumPoses(totalPose, robotPoses[index].times(weights[index]));
            }
        }
        return totalPose;
    }

    public void update(){

        for (int i = 0; i < tagVisible.length; i++) { tagVisible[i] = false; }
        scanTags3d();

        Pose3d robotPose3d = calcWeightedPose();
        poseConfidence = 0.0;

        for(int index = 0; index < tagConfidence.length; index++){
            if(tagVisible[index]){
                poseConfidence = Math.max(poseConfidence, tagConfidence[index]);
            }
            tagConfidence[index] -= CONFIDENCE_FORGET*dt;
        }

        estimatedRobotPose = new Pose2d(new Translation2d(robotPose3d.getX(), robotPose3d.getY()),
            new Rotation2d(robotPose3d.getRotation().getZ()));
        
        dt = timer.get()-lastT;
        lastT = timer.get();

        confidencePublisher.set(poseConfidence);
        double[] poseArray = {robotPose3d.getX(), robotPose3d.getY(), robotPose3d.getZ()};
        if(!(poseArray[0] == 0 && poseArray[1] == 0 && poseArray[2] == 0 )) estimatedPosePublisher.set(poseArray, 3);
    }

    public Pose2d getRobotPose2d(){
        return estimatedRobotPose;
    }
    public double getPoseConfidence(){
        return poseConfidence;
    }
    
}
