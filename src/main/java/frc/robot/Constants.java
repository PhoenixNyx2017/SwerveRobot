// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static double MAX_LIN_SPEED = 1.0; // meters/sec
    public static double MAX_ROT_SPEED = 1.0; // rad/sec

    // TODO: get true ID numbers
    public final static int LEFT_FRONT_DRIVE_ID = 1;
    public final static int LEFT_BACK_DRIVE_ID = 2;
    public final static int RIGHT_FRONT_DRIVE_ID = 3;
    public final static int RIGHT_BACK_DRIVE_ID = 4;

    public final static int LEFT_FRONT_TURN_ID = 11;
    public final static int LEFT_BACK_TURN_ID = 12;
    public final static int RIGHT_FRONT_TURN_ID = 13;
    public final static int RIGHT_BACK_TURN_ID = 14;

    public final static int LEFT_FRONT_ENCODER_ID = 21;
    public final static int LEFT_BACK_ENCODER_ID = 22;
    public final static int RIGHT_FRONT_ENCODER_ID = 23;
    public final static int RIGHT_BACK_ENCODER_ID = 24;

    // TODO
    public final static int ENCODER_TICKS_PER_ROTATION = 2048; // NEED NUMBER
    public final static double WHEEL_DIAMETER = 4.0;// inches

    public final static boolean FIELD_BASED = false;

    public final static int HUNDRED_MS_IN_SEC = 10;

    // TODO: get true constants
    public final static int JOYSTICK_LEFT_X_AXIS = 0;
    public final static int JOYSTICK_LEFT_Y_AXIS = 1;
    public final static int JOYSTICK_RIGHT_X_AXIS = 2;
    public final static int JOYSTICK_RIGHT_Y_AXIS = 3;

    // TODO: Math constants
    public final static double INCHES_2_METERS = 0.0254;

    public static int LEFT_FRONT_ANGLE_ENCODER_OFFSET = 0;
    public static int LEFT_BACK_ANGLE_ENCODER_OFFSET = 0;
    public static int RIGHT_FRONT_ANGLE_ENCODER_OFFSET = 0;
    public static int RIGHT_BACK_ANGLE_ENCODER_OFFSET = 0;

    public final static double TRACK_WIDTH = (22.0 - 2.625 - 2.625) * INCHES_2_METERS; // number in meters
    public final static double WHEEL_BASE = (22.0 - 2.625 - 2.625) * INCHES_2_METERS; // number in meters

    public final static String SUBSYSTEM_VERSION = "Test";

    public final static int DRIVER_JOYSTICK_ID = 0;
    public final static int OPERATOR_JOYSTICK_ID = 1;

    public final static int TRIANGLE = 0;
    public final static int SQUARE = 1;
    public final static int X_BUTTON = 2;

}
