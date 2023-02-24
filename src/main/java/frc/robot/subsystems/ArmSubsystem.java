package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Bezier;
import frc.robot.util.Statics;
import frc.robot.util.Vector;

public class ArmSubsystem extends SubsystemBase {

    public final PulleySubsystem m_pulleySubsystem;
    public final ElevatorSubsystem m_elevatorSubsystem;
    public final ManipulatorSubsystem m_manipulatorSubsystem;

    private final double ARM_JOINT_Y = 29;
    private final double ARM_JOINT_X = -14;

    private boolean enabled = false;

    private Bezier targetPath;
    private double followT = 0.0;
    private double followSpeed = 3.0; // Inches per second
    private double tSpeed = 0.0;

    public ArmSubsystem(PulleySubsystem pulleySubsystem, ElevatorSubsystem elevatorSybsystem, ManipulatorSubsystem manipulatorSubsystem){
        m_pulleySubsystem = pulleySubsystem;
        m_elevatorSubsystem = elevatorSybsystem;
        m_manipulatorSubsystem = manipulatorSubsystem;

        this.addChild(m_pulleySubsystem.getName(), m_pulleySubsystem);
        this.addChild(m_elevatorSubsystem.getName(), m_elevatorSubsystem);
        this.addChild(m_manipulatorSubsystem.getName(), m_manipulatorSubsystem);
    }

    public void periodic() {
        SmartDashboard.putString("Arm Position", getManipulator2dPosition().toString());

        if(targetPath != null && followT <= 1){
            double t = Statics.clamp(followT, 0.0, 1.0);
            Vector target = targetPath.at(t);
            setManipulator2dPosition(target.x, target.y);
            followT += tSpeed;
        }
    }

    public void followPath(Bezier path){
        targetPath = path;
        followT = path.nearestT(getManipulator2dPosition(), 0.01);
        tSpeed = targetPath.lengthEstimate(0.01)*followSpeed;
    }

    /* Raw Controls */
    public void setPulleyPower(double power){
        m_pulleySubsystem.setMotorPower(power);
    }
    public void setElevatorPower(double power){
        m_elevatorSubsystem.setMotorPower(power);
    }
    public void setManipulatorHingePower(double power){
        m_manipulatorSubsystem.setHingePower(power);
    }

    /* PID Controls */
    public void setPulleyTarget(double length){
        m_pulleySubsystem.setTargetInches(length);
    }

    public void setElevatorTarget(double position){
        m_elevatorSubsystem.setTarget(position);
    }

    public void setArmTargetAngle(double angle){
        m_elevatorSubsystem.setTargetArmAngle(angle);
    }

    public void setManipulatorHingeTarget(double angle){
        m_manipulatorSubsystem.setHingeTarget(angle);
    }

    public void setManipulator2dPosition(double x, double y){
        double _x = x-ARM_JOINT_X;
        double _y = y-ARM_JOINT_Y;
        double angle = Math.atan2(_y, _x);
        double length = Math.hypot(_x, _y);

        m_elevatorSubsystem.setTargetArmAngle(angle);
        m_pulleySubsystem.setExtensionLengthFromJoint(length);
    }

    public Vector getManipulator2dPosition(){
        double radius = m_pulleySubsystem.getExtensionLengthFromJoint();
        double theta = m_elevatorSubsystem.getArmAngle();

        // Arm space
        double x = radius*Math.cos(Math.toRadians(theta));
        double y = radius*Math.cos(Math.toRadians(theta));

        // Robot Space
        Vector position = new Vector(x+ARM_JOINT_X, y+ARM_JOINT_Y);
        return position;
    }

    public void updateAllPID(){
        if(isEnabled()){
            m_elevatorSubsystem.updatePID();
            m_manipulatorSubsystem.updateHingePID();
            m_pulleySubsystem.updatePID();
        }
    }

    public double getArmAngle(){
        return m_elevatorSubsystem.getArmAngle();
    }
    public double getElevatorPosition(){
        return m_elevatorSubsystem.getPosition();
    }
    public double getArmLength(){
        return m_pulleySubsystem.getExtensionLengthFromJoint();
    }
    public double getManipulatorHingeAngle(){
        return m_manipulatorSubsystem.getHingeAngle();
    }




    public boolean isArmAtSetpoint(){
        return m_elevatorSubsystem.isAtSetpoint() && m_pulleySubsystem.isAtSetpoint();
    }
    public boolean isManipulatorAtSetpoint(){
        return m_manipulatorSubsystem.isHingeAtSetpoint();
    }
    public boolean isAllAtSetpoint(){
        return isArmAtSetpoint() && isManipulatorAtSetpoint();
    }

    public void resetEncoders(){
        m_elevatorSubsystem.resetEncoder();
        m_pulleySubsystem.resetEncoder();
        m_manipulatorSubsystem.resetEncoder();
    }

    public void disable(){
        enabled = false;
        m_elevatorSubsystem.resetPID();
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
