// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 128.4;  // 51 (1/13/23) // -53 (10/26/22) // 51 (11/04/22)
    public static final boolean FRONT_LEFT_REVERSE_DRIVE = true;

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 2;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 1;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 12;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 139.8; // 156 (1/13/23) // 23.8 (10/24/22) // 156 (11/04/22)
    public static final boolean FRONT_RIGHT_REVERSE_DRIVE = false;

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 8; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 14; 
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = 256.5; // 218 (1/13/23) // -37 (10/26/2022) // 218 (11/04/22)
    public static final boolean BACK_LEFT_REVERSE_DRIVE = true;

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 5; 
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 6; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 11;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 247.2; // 215 (1/13/23) // -37 (10/31/22) // 215 (11/04/22)
    public static final boolean BACK_RIGHT_REVERSE_DRIVE = false;


    /****** Elevator ******/
    public static final int ELEVATOR_EXTENSION_MOTOR_LEFT = 9;
    public static final int ELEVATOR_EXTENSION_MOTOR_RIGHT = 10;
    public static final int ELEVATOR_EXTENSION_LIMIT = 1;

    /****** Manipulator ******/
    public static final int MANIPULATOR_ROLLER_MOTOR = 11;

    /****** Arm ******/
    public static final int SHOULDER_MOTOR = 9;
    public static final int ELBOW_MOTOR = 10;
    public static final int SHOULDER_LIMIT = 1;
    public static final int ELBOW_LIMIT = 2;
}
