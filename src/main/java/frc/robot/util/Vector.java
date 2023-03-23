package frc.robot.util;

import java.lang.Math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Vector {
    public double x;
    public double y;

    public Vector(double x, double y){
        this.x = x;
        this.y = y;
    }
    public Vector(){
        this.x = this.y = 0;
    }
    public static Vector fromAngle(double angle, double magnitude){
        return new Vector(Math.cos(angle)*magnitude, Math.sin(angle)*magnitude);
    }

    public void rotateByAngle(double angle){
        double tx = Math.cos(angle)*this.x - Math.sin(angle)*this.y;
        double ty = Math.sin(angle)*this.x + Math.cos(angle)*this.y;
        this.x = tx;
        this.y = ty;
    }
    public Vector add(Vector v){
        return new Vector(this.x+v.x, this.y+v.y);
    }
    public Vector subtract(Vector v){
        return new Vector(this.x-v.x, this.y-v.y);
    }
    public Vector copy(){
        return new Vector(this.x, this.y);
    }
    public void copyFrom(Vector v){
        this.x = v.x;
        this.y = v.y;
    }
    public double[] toArray(){
        double[] arr = {this.x, this.y};
        return arr;
    }
    public double magnitude(){
        return Math.hypot(this.x, this.y);
    }
    public Vector scale(double scale){
        return new Vector(this.x*scale, this.y*scale);
    }
    public double direction(){
        return Math.atan2(this.y, this.x);
    }
    public double dot(Vector v){
        return (this.x*v.x)+(this.y*v.y);
    }
    public Vector proj(Vector v){
        return this.scale(this.dot(v)/(this.dot(this)));
    }
    public Vector normalize(){
        double mag = this.magnitude();
        if(mag == 0) return new Vector(0, 0);
        return this.scale(1/mag);
    }

    public String toString(){
        return "["+x+", "+y+"]";
    }


    public static Vector fromArray(double[] arr){
        return new Vector(arr[0], arr[1]);
    }
    public static Vector fromTranslation(Translation2d t){
        return new Vector(t.getX(), t.getY());
    }
    public static Vector fromPose(Pose2d pose){
        return new Vector(pose.getX(), pose.getY());
    }
}
