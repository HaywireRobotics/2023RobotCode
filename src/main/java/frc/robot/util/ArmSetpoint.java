package frc.robot.util;

public class ArmSetpoint {
    public final Vector armPosition;
    public final double manipulatorAngle;

    public ArmSetpoint(Vector armPosition, double manipulatorAngle){
        this.armPosition = armPosition;
        this.manipulatorAngle = manipulatorAngle;
    }
}
