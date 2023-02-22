// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.util.Statics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
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

    public static final int MAX_SPEED = 1000;
    // OFFSET values changed on 1/14/23 to fix widebot conumdrum
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 3;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 4;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 13; 
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 126.1;  // 51 (1/13/23) // -53 (10/26/22) // 51 (11/04/22)
    public static final boolean FRONT_LEFT_REVERSE_DRIVE = true;

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 2;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 1;
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

    /****** Elevator ******/
    public static final int ELEVATOR_MOTOR = 9;
    public static final int ELEVATOR_EXTENSION_LIMIT = 1;

    /****** Manipulator ******/
    public static final int MANIPULATOR_ROLLER_MOTOR = 11;

    /****** Arm ******/
    public static final int PULLEY_MOTOR = 9;
    public static final int MANIPULATOR_HINGE_MOTOR = 10;

    /****** April Tags ******/
    //These need to be updated (and in meters)
    public static AprilTag[] aprilTags = {
        new AprilTag(1, Statics.poseToMeters(new Pose3d(new Translation3d(610.77, 42.19, 18.22), new Rotation3d(0, 0, Math.PI)))),
        new AprilTag(2, Statics.poseToMeters(new Pose3d(new Translation3d(610.77, 108.19, 18.22), new Rotation3d(0, 0, Math.PI)))),
        new AprilTag(3, Statics.poseToMeters(new Pose3d(new Translation3d(610.77, 174.19, 18.22), new Rotation3d(0, 0, Math.PI)))),
        new AprilTag(4, Statics.poseToMeters(new Pose3d(new Translation3d(636.96, 265.74, 27.38), new Rotation3d(0, 0, Math.PI)))),
        new AprilTag(5, Statics.poseToMeters(new Pose3d(new Translation3d(14.25, 265.74, 27.38), new Rotation3d(0, 0, 0)))),
        new AprilTag(6, Statics.poseToMeters(new Pose3d(new Translation3d(40.45, 147.19, 18.22), new Rotation3d(0, 0, 0)))),
        new AprilTag(7, Statics.poseToMeters(new Pose3d(new Translation3d(40.45, 108.19, 18.22), new Rotation3d(0, 0, 0)))),
        new AprilTag(8, Statics.poseToMeters(new Pose3d(new Translation3d(40.45, 42.19, 18.22), new Rotation3d(0, 0, 0))))
    };

    public static final Pose3d cameraPose = Statics.poseToMeters(new Pose3d(new Translation3d(6, 12, 14), new Rotation3d()));
}
