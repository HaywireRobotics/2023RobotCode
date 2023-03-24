package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public final class Statics {
    public static double clamp(double value, double min, double max){
        return Math.min(Math.max(value, min), max);
    }
    public static double map(double value, double oldMin, double oldMax, double newMin, double newMax) {
        return ((value - oldMin) / (oldMax - oldMin)) * (newMax - newMin) + newMin;
    }

    public static boolean withinError(double value, double target, double error){
        return value < target + error && value > target - error;
    }

    public static Pose3d poseToMeters(Pose3d pose){
        double mPi = 0.0254;
        return new Pose3d(new Translation3d(pose.getX()*mPi, pose.getY()*mPi, pose.getZ()*mPi), pose.getRotation());
    }
    public static Pose2d poseToMeters(Pose2d pose){
        double mPi = 0.0254;
        return new Pose2d(new Translation2d(pose.getX()*mPi, pose.getY()*mPi), pose.getRotation());
    }

    public static double[] normalize(double[] values){
        double maxVal = 0.0;
        double[] output = new double[values.length];
        for (double v : values) {
            if(v > maxVal) maxVal = v;
        }
        if (maxVal == 0) return values;

        for(int i = 0; i < values.length; i++){
            output[i] = values[i]/maxVal;
        }
        return output;
    }
    public static double[] normalizeSum(double[] values){
        double total = 0.0;
        double[] output = new double[values.length];
        for (double v : values) {
            total += v;
        }
        if (total == 0) return values;

        for(int i = 0; i < values.length; i++){
            output[i] = values[i]/total;
        }
        return output;
    }
    public static Pose3d sumPoses(Pose3d pose1, Pose3d pose2){
        return new Pose3d(pose1.getTranslation().plus(pose2.getTranslation()),
         pose1.getRotation().plus(pose2.getRotation()));
    }
    public static Pose2d sumPoses(Pose2d pose1, Pose2d pose2){
        return new Pose2d(pose1.getTranslation().plus(pose2.getTranslation()),
         pose1.getRotation().plus(pose2.getRotation()));
    }

    public static double applyDeadband(double x, double size) {
        double _x = Math.abs(x);
        double s1 = 1/(1-size);
        _x = _x*s1 - size*s1;
        return Math.max(_x, 0) * Math.signum(x);
    }

    public static double applySmoothing1D(double x, double s, double t) {
        double _x = Math.abs(x);
        double _xp = Math.pow(_x, 3)*(t+s-2)+Math.pow(_x, 2)*(-2*s-t+3)+s*_x;
        return _xp*Math.signum(x);
    }
    public static Vector applySmoothing2D(Vector p, double s, double t) {
        return p.normalize().scale(applySmoothing1D(p.magnitude(), s, t));
    }

    public static double angleDifference( double angle1, double angle2 )
    {
        double diff = ( angle2 - angle1 + 180 ) % 360 - 180;
        return diff < -180 ? diff + 360 : diff;
    }
}
