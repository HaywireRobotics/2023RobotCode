package frc.robot.util;

public class Bezier {
    private final Vector p1;
    private final Vector h1;
    private final Vector p2;
    private final Vector h2;

    public Bezier(Vector p1, Vector h1, Vector p2, Vector h2){
        this.p1 = p1;
        this.p2 = p2;
        this.h1 = h1;
        this.h2 = h2;
    }

    public Vector at(double t){
        double tt = t*t;
        double ttt = tt*t;

        double q1 = -ttt + 3.0*tt -3.0*t + 1;
        double q2 = 3.0*ttt - 6*tt + 3.0*t;
        double q3 = -3.0*ttt + 3.0*tt;
        double q4 = ttt;

        double tx = p1.x*q1 + h1.x*q2 + h2.x*q3 + p2.x*q4;
        double ty = p1.y*q1 + h1.y*q2 + h2.y*q3 + p2.y*q4;

        return new Vector(tx, ty);
    }
    public Vector gradientAt(double t){
        double tt = t*t;

        double q1 = -3.0*tt + 6.0*t -3.0;
        double q2 = 9.0*tt - 12.0*t + 3.0;
        double q3 = -9.0*tt + 6.0*t;
        double q4 = 3.0*tt;

        double tx = p1.x*q1 + h1.x*q2 + h2.x*q3 + p2.x*q4;
        double ty = p1.y*q1 + h1.y*q2 + h2.y*q3 + p2.y*q4;

        return new Vector(tx, ty);
    }

    public double nearestT(Vector target, double resolution){
        double bestT = 0;
        double bestDist2 = Double.MAX_VALUE;
        for(double t = 0; t < 1; t+=resolution){
            Vector point = at(t);
            Vector error = target.subtract(point);
            double distance2 = error.magnitude();
            // double distance2 = error.x*error.x + error.y*error.y;
            if(distance2 < bestDist2){
                bestDist2 = distance2;
                bestT = t;
            }
        }
        return bestT;
    }
    public double nearestT(Vector target, double start, double stride, double scanSize){
        double bestT = 0;
        double bestDist2 = Double.MAX_VALUE;
        for(double t = Statics.clamp(start, 0, 1); Math.abs(t-start) < Math.abs(scanSize) && t >= 0 && t <= 1; t+=stride){
            Vector point = at(t);
            Vector error = target.subtract(point);
            double distance2 = error.x*error.x + error.y*error.y;
            if(distance2 < bestDist2){
                bestDist2 = distance2;
                bestT = t;
            }
        }
        return bestT;
    }
    public double lengthEstimate(double resolution){
        double length = 0;
        Vector last = at(0);
        for(double t = resolution; t <= 1; t+=resolution){
            Vector point = at(t);
            Vector error = point.subtract(last);
            length += Math.sqrt(error.x*error.x + error.y*error.y);
            last = point;
        }
        return length;
    }

    public Bezier getReversed(){
        return new Bezier(p2, h2, p1, h1);
    }
}
