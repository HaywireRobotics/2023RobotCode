package frc.robot.util;

public final class Statics {
    public static double clamp(double value, double min, double max){
        return Math.min(Math.max(value, min), max);
    }

    public static boolean withinError(double value, double target, double error){
        return value < target + error && value > target - error;
    }
}
