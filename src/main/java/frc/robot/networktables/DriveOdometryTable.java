package frc.robot.networktables;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DriveOdometryTable {
    private NetworkTable table;

    private DoublePublisher xPositionPublisher;
    private DoublePublisher yPositionPublisher;
    private DoublePublisher headingPublisher;

    public DriveOdometryTable(NetworkTableInstance networkTableInstance) {
        this.table = networkTableInstance.getTable("drive");

        this.xPositionPublisher = table.getDoubleTopic("positionX").publish();
        this.yPositionPublisher = table.getDoubleTopic("positionY").publish();
        this.headingPublisher = table.getDoubleTopic("heading").publish();
    }

    public void publishPose(Pose2d pose){
        this.xPositionPublisher.set(pose.getX());
        this.yPositionPublisher.set(pose.getY());
        this.headingPublisher.set(pose.getRotation().getDegrees());
    }
}
