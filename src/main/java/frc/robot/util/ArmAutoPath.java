package frc.robot.util;

public class ArmAutoPath {
    public final Bezier path;
    public final double manipulatorAngle;
    
    public ArmAutoPath(Bezier path, double manipulatorAngle){
        this.path = path;
        this.manipulatorAngle = manipulatorAngle;
    }
}
