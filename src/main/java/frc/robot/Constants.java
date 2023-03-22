// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.util.ArmAutoPath;
import frc.robot.util.ArmSetpoint;
import frc.robot.util.Bezier;
import frc.robot.util.Statics;
import frc.robot.util.Vector;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double fieldHeadingMag = 0.0;
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 20.5;
    public static final double DRIVETRAIN_TRACKLENGTH_METERS = 25.5;
    public static final double DRIVE_THETA_OFFSET = Math.toDegrees(Math.atan(DRIVETRAIN_TRACKLENGTH_METERS/DRIVETRAIN_TRACKWIDTH_METERS));
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    // public static final double DRIVETRAIN_WHEELBASE_METERS = 0.48895;

    public static final int MAX_SPEED = 3000; // 1500 as of 3/1/23
    // OFFSET values changed on 1/14/23 to fix widebot conumdrum
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 3;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 4;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 13; 
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 126.1;  // 51 (1/13/23) // -53 (10/26/22) // 51 (11/04/22)
    public static final boolean FRONT_LEFT_REVERSE_DRIVE = true;

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 2;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 17;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 12;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 141.4; //139.8 // 156 (1/13/23) // 23.8 (10/24/22) // 156 (11/04/22)
    public static final boolean FRONT_RIGHT_REVERSE_DRIVE = false;

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 8; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 14; 
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = 243.8; // 218 (1/13/23) // -37 (10/26/2022) // 218 (11/04/22)
    public static final boolean BACK_LEFT_REVERSE_DRIVE = true;

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 5; 
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 6; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 11;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 246.2; // 215 (1/13/23) // -37 (10/31/22) // 215 (11/04/22)
    public static final boolean BACK_RIGHT_REVERSE_DRIVE = false;

    public static final double STEER_MOTOR_GEAR_RATIO = 12.8 / 1;
    public static final double DRIVE_MOTOR_GEAR_RATIO = 8.14 / 1; // could potentially be 6.75:1 depending on if it is L1 of L2
                                                                  // see https://www.swervedrivespecialties.com/products/mk4-swerve-module?variant=39376675012721
    public static final double WHEEL_DIAMETER = 0.1016; // 4 inches

    public static final double kMaxSpeedMetersPerSecond = 1.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.3;
    public static final TrajectoryConfig TRAJECTORY_CONFIG = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,Constants.kMaxAccelerationMetersPerSecondSquared);

    /****** Elevator ******/
    public static final int ELEVATOR_MOTOR = 9;
    public static final int ELEVATOR_TOP_LIMIT_SWITCH = 0;
    public static final int ELEVATOR_ENCODER = 2;

    /****** Manipulator ******/
    public static final int MANIPULATOR_ROLLER_MOTOR = 15;
    public static final int MANIPULATOR_HINGE_MOTOR = 16;

    /****** Arm ******/
    public static final int PULLEY_MOTOR = 10;
    public static final int PULLEY_RETRACTED_LIMIT_SWITCH = 1;

    /****** April Tags ******/
    //These need to be updated (and in meters)
    public static AprilTag[] aprilTags = {
        new AprilTag(1, Statics.poseToMeters(new Pose3d(new Translation3d(610.77, 42.19, -18.22), new Rotation3d(0, 0, Math.PI)))),
        new AprilTag(2, Statics.poseToMeters(new Pose3d(new Translation3d(610.77, 108.19, -18.22), new Rotation3d(0, 0, Math.PI)))),
        new AprilTag(3, Statics.poseToMeters(new Pose3d(new Translation3d(610.77, 174.19, -18.22), new Rotation3d(0, 0, Math.PI)))),
        new AprilTag(4, Statics.poseToMeters(new Pose3d(new Translation3d(636.96, 265.74, -27.38), new Rotation3d(0, 0, Math.PI)))),
        new AprilTag(5, Statics.poseToMeters(new Pose3d(new Translation3d(14.25, 265.74, -27.38), new Rotation3d(0, 0, 0)))),
        new AprilTag(6, Statics.poseToMeters(new Pose3d(new Translation3d(40.45, 147.19, -18.22), new Rotation3d(0, 0, 0)))),
        new AprilTag(7, Statics.poseToMeters(new Pose3d(new Translation3d(40.45, 108.19, -18.22), new Rotation3d(0, 0, 0)))),
        new AprilTag(8, Statics.poseToMeters(new Pose3d(new Translation3d(40.45, 42.19, -18.22), new Rotation3d(0, 0, 0))))
    };

    // to set y, set x, to set x set y, to set z set z
    // public static final Pose3d cameraPose = Statics.poseToMeters(new Pose3d(new Translation3d(10, 5, 24), new Rotation3d(0, 0, 0))); // y-up System
    public static final Pose3d cameraPose = Statics.poseToMeters(new Pose3d(new Translation3d(0, 0, 22.7), new Rotation3d(0, 0, 0))); // y-up System

    /****** Arm Auto Paths ******/
    // public static enum ArmSetpoints {
    //     CONE_HEIGH,
    //     CONE_MID,
    //     CUBE_HIGH,
    //     CUBE_MID,
    //     SUBSTATION,
    //     GROUND
    // }
    public static final class ArmSetpoints {
        public static final ArmSetpoint CONE_HIGH = new ArmSetpoint(new Vector(DriveSetpoints.scoringDistance+29, 52), 90);
        public static final ArmSetpoint CONE_MID = new ArmSetpoint(new Vector(DriveSetpoints.scoringDistance+10, 42), 100);
        public static final ArmSetpoint CUBE_HIGH = new ArmSetpoint(new Vector(DriveSetpoints.scoringDistance+30, 35), 90);
        public static final ArmSetpoint CUBE_MID = new ArmSetpoint(new Vector(DriveSetpoints.scoringDistance+7, 21), 100);
        public static final ArmSetpoint SUBSTATION = new ArmSetpoint(new Vector(DriveSetpoints.substationDistance, 125), 140);
        public static final ArmSetpoint GROUND = new ArmSetpoint(new Vector(12, 10.5), 70);
        public static final ArmSetpoint STOWED = new ArmSetpoint(new Vector(12, 9.5), 0);
    }
    public static final class BezierHandles{
        public static final Vector leaveStowed = new Vector(0, 8);
        public static final Vector leaveHigh = new Vector(-8, 5);
        public static final Vector leaveMid = new Vector(-8, 3);
        public static final Vector leaveSubstation = new Vector(-5, 10);

    }
    public static final class ArmSetpointPaths {
        public static final ArmAutoPath CONE_HIGH = new ArmAutoPath(new Bezier(ArmSetpoints.STOWED.armPosition, ArmSetpoints.STOWED.armPosition.add(BezierHandles.leaveStowed),
                                                                     ArmSetpoints.CONE_HIGH.armPosition, ArmSetpoints.CONE_HIGH.armPosition.add(BezierHandles.leaveHigh)),
                                                                     ArmSetpoints.CONE_HIGH.manipulatorAngle, 0.5);
        public static final ArmAutoPath CONE_MID = new ArmAutoPath(new Bezier(ArmSetpoints.STOWED.armPosition, ArmSetpoints.STOWED.armPosition.add(BezierHandles.leaveStowed),
                                                                        ArmSetpoints.CONE_MID.armPosition, ArmSetpoints.CONE_MID.armPosition.add(BezierHandles.leaveMid)),
                                                                        ArmSetpoints.CONE_MID.manipulatorAngle, 0.5);
        public static final ArmAutoPath CONE_HIGH_TO_MID = new ArmAutoPath(new Bezier(ArmSetpoints.CONE_HIGH.armPosition, ArmSetpoints.CONE_HIGH.armPosition.add(new Vector(-3, 6)),
                                                                        ArmSetpoints.CONE_MID.armPosition, ArmSetpoints.CONE_MID.armPosition.add(new Vector(-2, 8))),
                                                                        ArmSetpoints.CONE_MID.manipulatorAngle, 0.5);
        public static final ArmAutoPath CUBE_HIGH = new ArmAutoPath(new Bezier(ArmSetpoints.STOWED.armPosition, ArmSetpoints.STOWED.armPosition.add(BezierHandles.leaveStowed),
                                                                        ArmSetpoints.CUBE_HIGH.armPosition, new Vector(0, 0)),
                                                                        ArmSetpoints.CUBE_HIGH.manipulatorAngle, 0.5);
        public static final ArmAutoPath CUBE_MID = new ArmAutoPath(new Bezier(ArmSetpoints.STOWED.armPosition, ArmSetpoints.STOWED.armPosition.add(BezierHandles.leaveStowed),
                                                                        ArmSetpoints.CUBE_MID.armPosition, new Vector(0, 0)),
                                                                        ArmSetpoints.CUBE_MID.manipulatorAngle, 0.5);
        public static final ArmAutoPath SUBSTATION = new ArmAutoPath(new Bezier(ArmSetpoints.STOWED.armPosition, ArmSetpoints.STOWED.armPosition.add(BezierHandles.leaveStowed),
                                                                        ArmSetpoints.SUBSTATION.armPosition, ArmSetpoints.SUBSTATION.armPosition.add(BezierHandles.leaveSubstation)),
                                                                        ArmSetpoints.SUBSTATION.manipulatorAngle, 0.95);
        
        public static final ArmAutoPath GROUND = new ArmAutoPath(new Bezier(ArmSetpoints.CONE_HIGH.armPosition, ArmSetpoints.CONE_HIGH.armPosition.add(BezierHandles.leaveHigh),
                                                                        ArmSetpoints.GROUND.armPosition, ArmSetpoints.GROUND.armPosition.add(BezierHandles.leaveStowed)),
                                                                        ArmSetpoints.GROUND.manipulatorAngle, 0.5);
        
        public static final ArmAutoPath STOW = new ArmAutoPath(new Bezier(ArmSetpoints.CONE_HIGH.armPosition, ArmSetpoints.CONE_HIGH.armPosition.add(BezierHandles.leaveHigh),
                                                                        ArmSetpoints.STOWED.armPosition, ArmSetpoints.STOWED.armPosition.add(BezierHandles.leaveStowed)),
                                                                        ArmSetpoints.STOWED.manipulatorAngle);
        
        public static ArmAutoPath getPathForScorePosition(ScorePositions scorePosition) {
            switch (scorePosition) {
                case CONE_HIGH:
                    return CONE_HIGH;
                case CUBE_HIGH:
                    return CUBE_HIGH;
                case CONE_MID:
                    return CONE_MID;
                case CUBE_MID:
                    return CUBE_MID;
                case GROUND:
                    return GROUND;
                case SUBSTATION:
                    return SUBSTATION;
                case STOW:
                    return STOW;
                default:
                    return GROUND;
            }
        }
    }

    /*
     * node y positions relative center of the field
     * 38.5
     * -5.5
     * -27.5
     * -71.5
     * -93.5
     * -137.5 (20.19 in frc coordinates)
     * frc system coordinates are 
     * 196.19
     * 152.19
     * 130.19
     * 86.19
     * 64.19
     * 20.19
     */

    public static enum Alliances {
        BLUE,
        RED
    }
    public static enum ScorePositions {
        CONE_HIGH,
        CONE_MID,
        CUBE_HIGH,
        CUBE_MID,
        GROUND,
        SUBSTATION,
        STOW
    }
    public static enum ScoreRows {
        HIGH,
        MID,
        LOW,
    }

    public static final class DriveSetpoints {
        private static final double scoringDistance = 24;
        private static final double substationDistance = 42;
        public static final Pose2d[] BlueGrid = {
            Statics.poseToMeters(new Pose2d(new Translation2d(56+scoringDistance, 196.19), new Rotation2d(Math.PI))),
            Statics.poseToMeters(new Pose2d(new Translation2d(56+scoringDistance, 152.19), new Rotation2d(Math.PI))),
            Statics.poseToMeters(new Pose2d(new Translation2d(56+scoringDistance, 130.19), new Rotation2d(Math.PI))),
            Statics.poseToMeters(new Pose2d(new Translation2d(56+scoringDistance, 86.19), new Rotation2d(Math.PI))),
            Statics.poseToMeters(new Pose2d(new Translation2d(56+scoringDistance, 64.19), new Rotation2d(Math.PI))),
            Statics.poseToMeters(new Pose2d(new Translation2d(56+scoringDistance, 20.19), new Rotation2d(Math.PI)))
        };
        public static final Pose2d[] RedGrid = {
            Statics.poseToMeters(new Pose2d(new Translation2d(651-scoringDistance, 196.19), new Rotation2d(0))),
            Statics.poseToMeters(new Pose2d(new Translation2d(651-scoringDistance, 152.19), new Rotation2d(0))),
            Statics.poseToMeters(new Pose2d(new Translation2d(651-scoringDistance, 130.19), new Rotation2d(0))),
            Statics.poseToMeters(new Pose2d(new Translation2d(651-scoringDistance, 86.19), new Rotation2d(0))),
            Statics.poseToMeters(new Pose2d(new Translation2d(651-scoringDistance, 64.19), new Rotation2d(0))),
            Statics.poseToMeters(new Pose2d(new Translation2d(651-scoringDistance, 20.19), new Rotation2d(0)))
        };

        public static final Pose2d[] BlueSubstation = {
            Statics.poseToMeters(new Pose2d(new Translation2d(14+substationDistance, 296), new Rotation2d(Math.PI))),
            Statics.poseToMeters(new Pose2d(new Translation2d(14+substationDistance, 232), new Rotation2d(Math.PI))),
        };
        public static final Pose2d[] RedSubstation = {
            Statics.poseToMeters(new Pose2d(new Translation2d(637-substationDistance, 296), new Rotation2d(0))),
            Statics.poseToMeters(new Pose2d(new Translation2d(637-substationDistance, 232), new Rotation2d(0))),
        };

        public static Pose2d getGridTargetPose(Alliances alliance, int column){
            if(alliance == Alliances.BLUE){
                return BlueGrid[column];
            } else {
                return RedGrid[column];
            }
        }
    }

    public static enum GamePieces {
        CONE,
        CUBE,
        NONE
    }
}
