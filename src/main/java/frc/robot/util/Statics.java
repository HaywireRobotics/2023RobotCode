package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public final class Statics {
    public static double clamp(double value, double min, double max){
        return Math.min(Math.max(value, min), max);
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
        return new Pose3d(sumTranslation(pose1.getTranslation(), pose2.getTranslation()),
         sumRotations(pose1.getRotation(), pose2.getRotation()));
    }
    public static Rotation3d sumRotations(Rotation3d r1, Rotation3d r2){
        return new Rotation3d(r1.getX()+r2.getX(), r1.getY()+r2.getY(), r1.getZ()+r2.getZ());
    }
    public static Translation3d sumTranslation(Translation3d t1, Translation3d t2){
        return new Translation3d(t1.getX()+t2.getX(), t1.getY()+t2.getY(), t1.getZ()+t2.getZ());
    }
}
