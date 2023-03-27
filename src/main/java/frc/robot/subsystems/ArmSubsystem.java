package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.AutoArmToSetpoint;
import frc.robot.util.ArmAutoPath;
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
    public double followT = 0.0;
    private double followSpeed = 0.7; // 0.35 (3/24/23) // Inches per second
    private double tSpeed = 0.0;

    private final double kF = 0.3;

    public ArmSubsystem(PulleySubsystem pulleySubsystem, ElevatorSubsystem elevatorSubsystem, ManipulatorSubsystem manipulatorSubsystem){
        m_pulleySubsystem = pulleySubsystem;
        m_elevatorSubsystem = elevatorSubsystem;
        m_manipulatorSubsystem = manipulatorSubsystem;

        this.addChild(m_pulleySubsystem.getName(), m_pulleySubsystem);
        this.addChild(m_elevatorSubsystem.getName(), m_elevatorSubsystem);
        this.addChild(m_manipulatorSubsystem.getName(), m_manipulatorSubsystem);
    }

    public void periodic() {
        SmartDashboard.putString("Arm Position", getManipulator2dPosition().toString());
        SmartDashboard.putNumber("Arm t", followT);
        

        if(targetPath != null && followT <= 1){
            double t = Statics.clamp(followT, 0.0, 1.0);
            Vector target = targetPath.at(t);
            Vector tangent = targetPath.gradientAt(t).normalize().scale(tSpeed*kF);
            if(t < 0.95) target = target.add(tangent);
            SmartDashboard.putString("Arm Goal", target.toString());
            setManipulator2dPosition(target.x, target.y);
            followT += tSpeed*(1/(getManipulator2dPosition().subtract(target).magnitude())+1);
        }
    }

    public void followPath(Bezier path){
        targetPath = path;
        followT = path.nearestT(getManipulator2dPosition(), 0.01);
        tSpeed = followSpeed/targetPath.lengthEstimate(0.01);
    }

    public void endPath(){
        targetPath = null;
        followT = 0.0;
        tSpeed = 0.0;
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
        double angle = Math.toDegrees(Math.atan(_y/_x));
        double length = Math.hypot(_x, _y);

        m_elevatorSubsystem.setTargetArmAngle(angle);
        m_pulleySubsystem.setExtensionLengthFromJoint(length);
    }

    public Vector getManipulator2dPosition(){
        double radius = m_pulleySubsystem.getExtensionLengthFromJoint();
        double theta = m_elevatorSubsystem.getArmAngle();

        // Arm space
        double x = radius*Math.cos(Math.toRadians(theta));
        double y = radius*Math.sin(Math.toRadians(theta));

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

    public void stabilizeManipulator() {
        m_manipulatorSubsystem.stabilize();
    }
    public void stabilizeArm() {
        m_elevatorSubsystem.stabilize();
        m_pulleySubsystem.stabilize();
    }
    public void stabilizeAll() {
        stabilizeManipulator();
        stabilizeArm();
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
    public double getArmRawLength(){
        return m_pulleySubsystem.getPositionInches();
    }
    public double getManipulatorHingeAngle(){
        return m_manipulatorSubsystem.getHingeAngle();
    }

    public Command collapseArmCommand() {
        return Commands.parallel(
            new InstantCommand(() -> setPulleyTarget(0)),
            new InstantCommand(() -> setElevatorTarget(1.5)),
            m_manipulatorSubsystem.setHingeTargetCommand(5)
        );
    }

    private boolean subsystemsAtSetpoints(){
        return m_elevatorSubsystem.isAtSetpoint() && m_pulleySubsystem.isAtSetpoint();
    }
    public boolean isArmAtSetpoint(){
        if(targetPath != null && subsystemsAtSetpoints()){
            return true;
        }
        else{
            return subsystemsAtSetpoints();
        }
    }
    public boolean isManipulatorAtSetpoint(){
        return m_manipulatorSubsystem.isHingeAtSetpoint();
    }
    public boolean isAllAtSetpoint(){
        return isArmAtSetpoint() && isManipulatorAtSetpoint();
    }
    public BooleanSupplier isAllAtSetpointBooleanSupplier() {
        BooleanSupplier sup = () -> isAllAtSetpoint();
        return sup;
    }

    public void resetEncoders(){
        m_elevatorSubsystem.resetEncoder();
        m_pulleySubsystem.resetEncoder();
        m_manipulatorSubsystem.resetEncoder();
    }

    public Command smartSetpointCommand(Constants.ScoreRows row){
        Constants.GamePieces gamePiece = m_manipulatorSubsystem.getGamePiece();
        ArmAutoPath path;
        Constants.SetpointPositions scorePosition = Constants.SetpointPositions.GROUND;
    
        if(row == Constants.ScoreRows.HIGH){
          if(gamePiece == Constants.GamePieces.CUBE){
            scorePosition = Constants.SetpointPositions.CUBE_HIGH;
          } else {
            scorePosition = Constants.SetpointPositions.CONE_HIGH;
          }
        } else if (row == Constants.ScoreRows.MID){
          if(gamePiece == Constants.GamePieces.CUBE){
            scorePosition = Constants.SetpointPositions.CUBE_MID;
          } else {
            scorePosition = Constants.SetpointPositions.CONE_MID;
          }
        }else{
          scorePosition = Constants.SetpointPositions.GROUND;
        }
    
        path = Constants.ArmSetpointPaths.getPathForSetpointPosition(scorePosition);
        return new AutoArmToSetpoint(this, path);
      }
      public Command adaptiveSetpointCommand(Constants.SetpointPositions scorePosition){
        ArmAutoPath path;
        Vector armPosition = this.getManipulator2dPosition();
        double distanceToHigh = Constants.ArmSetpoints.CONE_HIGH.armPosition.subtract(armPosition).magnitude();
        if(scorePosition == Constants.SetpointPositions.CONE_MID && distanceToHigh < 5){
          path = Constants.ArmSetpointPaths.CONE_HIGH_TO_MID;
        }if(scorePosition == Constants.SetpointPositions.STOW && armPosition.y < 10){
            path = Constants.ArmSetpointPaths.FLOOR_STOW;
        }else{
          path = Constants.ArmSetpointPaths.getPathForSetpointPosition(scorePosition);
        }
        return new AutoArmToSetpoint(this, path);
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
