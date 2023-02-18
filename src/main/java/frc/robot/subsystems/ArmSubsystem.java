package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    private final PulleySubsystem m_pulleySubsystem;
    private final ElevatorSybsystem m_elevatorSybsystem;
    private final ManipulatorSubsystem m_manipulatorSubsystem;

    public ArmSubsystem(PulleySubsystem pulleySubsystem, ElevatorSybsystem elevatorSybsystem, ManipulatorSubsystem manipulatorSubsystem){
        m_pulleySubsystem = pulleySubsystem;
        m_elevatorSybsystem = elevatorSybsystem;
        m_manipulatorSubsystem = manipulatorSubsystem;

        this.addChild(m_pulleySubsystem.getName(), m_pulleySubsystem);
        this.addChild(m_elevatorSybsystem.getName(), m_elevatorSybsystem);
        this.addChild(m_manipulatorSubsystem.getName(), m_manipulatorSubsystem);
    }

    public void setPulleyTarget(double length){
        m_pulleySubsystem.setPulleyTargetMeters(length);
    }

    public void setElevatorTarget(double position){
        m_elevatorSybsystem.setTarget(position);
    }

    public void setArmTargetAngle(double angle){
        m_elevatorSybsystem.setTargetArmAngle(angle);
    }

    public void setManipulator2dPosition(double x, double y){
        
    }
    
}
