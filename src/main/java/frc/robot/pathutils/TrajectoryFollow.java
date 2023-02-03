// package frc.robot.pathutils;

// import frc.robot.util.Vector;

// public class TrajectoryFollow {

//     private Spline targetPath;
//     private double virtualT = 0;
//     private double forwardScan = 0.1;

//     public TrajectoryFollow(){

//     }

//     private void calculateTargets(Vector position, Vector velocity){
//         double closesetT = targetPath.getclosestT(virtualT);
//         Vector closePoint = targetPath.pointAtT(closesetT);
//         Vector futurePoint = targetPath.pointAtT(closesetT);
//         Vector positionError = position.subtract(closePoint);

//         Vector closeNormal = targetPath.getNormalAtT(closesetT);
//         Vector closeTangent = new Vector(1-closeNormal.y, 1-closeNormal.x);
//         Vector velInNormal = closeNormal.proj(velocity);
//         Vector posErrorInTangent = closeTangent.proj(positionError);
//     }
// }
