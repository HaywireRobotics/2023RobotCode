package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Bezier;
import frc.robot.util.Statics;
import frc.robot.util.Vector;

public class ArmSubsystem extends SubsystemBase {

    private final PulleySubsystem m_pulleySubsystem;
    private final ElevatorSybsystem m_elevatorSybsystem;
    private final ManipulatorSubsystem m_manipulatorSubsystem;

    private final double ARM_JOINT_Y = 29;
    private final double ARM_JOINT_X = -14;

    private boolean enabled = false;

    private Bezier targetPath;
    private double followT = 0.0;
    private double followSpeed = 0.001;

    public ArmSubsystem(PulleySubsystem pulleySubsystem, ElevatorSybsystem elevatorSybsystem, ManipulatorSubsystem manipulatorSubsystem){
        m_pulleySubsystem = pulleySubsystem;
        m_elevatorSybsystem = elevatorSybsystem;
        m_manipulatorSubsystem = manipulatorSubsystem;

        this.addChild(m_pulleySubsystem.getName(), m_pulleySubsystem);
        this.addChild(m_elevatorSybsystem.getName(), m_elevatorSybsystem);
        this.addChild(m_manipulatorSubsystem.getName(), m_manipulatorSubsystem);
    }

    public void periodic() {
        SmartDashboard.putString("Arm Position", getManipulator2dPosition().toString());

        if(targetPath != null && followT <= 1){
            double t = Statics.clamp(followT, 0.0, 1.0);
            Vector target = targetPath.at(t);
            setManipulator2dPosition(target.x, target.y);
            followT += followSpeed;
        }
    }

    public void followPath(Bezier path){
        targetPath = path;
        followT = path.nearestT(getManipulator2dPosition(), 0.01);
    }

    /* Raw Controlls */
    public void setPulleyPower(double power){
        m_pulleySubsystem.setMotorPower(power);
    }
    public void setElevatorPower(double power){
        m_elevatorSybsystem.setMotorPower(power);
    }
    public void setManipulatorHingePower(double power){
        m_manipulatorSubsystem.setHingePower(power);
    }

    /* PID Controlls */
    public void setPulleyTarget(double length){
        m_pulleySubsystem.setTargetInches(length);
    }

    public void setElevatorTarget(double position){
        m_elevatorSybsystem.setTarget(position);
    }

    public void setArmTargetAngle(double angle){
        m_elevatorSybsystem.setTargetArmAngle(angle);
    }

    public void setManipulator2dPosition(double x, double y){
        double _x = x-ARM_JOINT_X;
        double _y = y-ARM_JOINT_Y;
        double angle = Math.atan2(_y, _x);
        double length = Math.hypot(_x, _y);

        m_elevatorSybsystem.setTargetArmAngle(angle);
        m_pulleySubsystem.setExtensionLengthFromJoint(length);
    }

    public Vector getManipulator2dPosition(){
        double radius = m_pulleySubsystem.getExtensionLengthFromJoint();
        double theta = m_elevatorSybsystem.getArmAngle();

        // Arm space
        double x = radius*Math.cos(Math.toRadians(theta));
        double y = radius*Math.cos(Math.toRadians(theta));

        // Robot Space
        Vector postion = new Vector(x+ARM_JOINT_X, y+ARM_JOINT_Y);
        return postion;
    }

    public void updateAllPID(){
        if(isEnabled()){
            m_elevatorSybsystem.updatePID();
            m_manipulatorSubsystem.updateHingePID();
            m_pulleySubsystem.updatePID();
        }
    }

    public void disable(){
        enabled = false;
        m_elevatorSybsystem.resetPID();
        m_manipulatorSubsystem.resetPID();
        m_pulleySubsystem.resetPID();
    }
    public void enable(){
        enabled = true;
    }
    public boolean isEnabled(){
        return enabled;
    }
    
}
