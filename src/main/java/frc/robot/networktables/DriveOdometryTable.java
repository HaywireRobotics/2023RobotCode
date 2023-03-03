package frc.robot.networktables;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveOdometryTable {
    private NetworkTable table;

    private DoublePublisher xPositionPublisher;
    private DoublePublisher yPositionPublisher;
    private DoublePublisher headingPublisher;
    
    private BooleanPublisher centricPublisher;

    private Field2d field2d;

    private DrivetrainSubsystem m_drivetrainSubsystem;

    public DriveOdometryTable(NetworkTableInstance networkTableInstance, DrivetrainSubsystem drivetrainSubsystem) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;

        table = networkTableInstance.getTable("drive");
        field2d = new Field2d();
        SmartDashboard.putData("Field", field2d);

        xPositionPublisher = table.getDoubleTopic("positionX").publish();
        yPositionPublisher = table.getDoubleTopic("positionY").publish();
        headingPublisher = table.getDoubleTopic("heading").publish();
        centricPublisher = table.getBooleanTopic("field centric").publish();

        
    }

    public void publishData(){
        Pose2d pose = m_drivetrainSubsystem.getPose();
        field2d.setRobotPose(pose);

        xPositionPublisher.set(pose.getX());
        yPositionPublisher.set(pose.getY());
        headingPublisher.set(pose.getRotation().getDegrees());
        centricPublisher.set(m_drivetrainSubsystem.field_centric_drive);
    }
}
