package frc.robot.util;

import java.net.BindException;

public class ArmAutoPath {
    public final Bezier path;
    public final double manipulatorAngle;
    public final double flipTime;

    public ArmAutoPath(Bezier path, double manipulatorAngle) {
        this(path, manipulatorAngle, 0.0);
    }
    
    public ArmAutoPath(Bezier path, double manipulatorAngle, double flipTime){
        this.path = path;
        this.manipulatorAngle = manipulatorAngle;
        this.flipTime = flipTime;
    }
}
