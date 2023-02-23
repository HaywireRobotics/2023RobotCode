package frc.robot.networktables;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.ArmSubsystem;

public class ArmTable {
    private NetworkTable table;

    private DoublePublisher xPositionPublisher;
    private DoublePublisher yPositionPublisher;
    private DoublePublisher armAnglePublisher;
    private DoublePublisher armLengthPublisher;
    private DoublePublisher elevatorHeightPublisher;
    private DoublePublisher manipulatorAnglePublisher;

    private ArmSubsystem m_armSubsystem;

    public ArmTable(NetworkTableInstance networkTableInstance, ArmSubsystem armSubsystem) {
        table = networkTableInstance.getTable("drive");
        m_armSubsystem = armSubsystem;

        xPositionPublisher = table.getDoubleTopic("positionX").publish();
        yPositionPublisher = table.getDoubleTopic("positionY").publish();
        armAnglePublisher = table.getDoubleTopic("armAngle").publish();
        armLengthPublisher = table.getDoubleTopic("armLength").publish();
        elevatorHeightPublisher = table.getDoubleTopic("elevatorHeight").publish();
        manipulatorAnglePublisher = table.getDoubleTopic("manipulatorAngle").publish();
    }

    public void publishData(){
        xPositionPublisher.set(m_armSubsystem.getManipulator2dPosition().x);
        yPositionPublisher.set(m_armSubsystem.getManipulator2dPosition().y);
        armAnglePublisher.set(m_armSubsystem.getArmAngle());
        armLengthPublisher.set(m_armSubsystem.getArmLength());
        elevatorHeightPublisher.set(m_armSubsystem.getElevatorPosition());
        manipulatorAnglePublisher.set(m_armSubsystem.getManipulatorHingeAngle());
    }
    
}
