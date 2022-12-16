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

    public static double MAX_LIN_SPEED = 0.5; // meters/sec
    public static double MAX_ROT_SPEED = 1.0; // rad/sec

    // TODO: get true ID numbers
    public final static int LEFT_FRONT_DRIVE_ID = 10;
    public final static int LEFT_BACK_DRIVE_ID = 30;
    public final static int RIGHT_FRONT_DRIVE_ID = 20;
    public final static int RIGHT_BACK_DRIVE_ID = 40;

    public final static int LEFT_FRONT_TURN_ID = 11;
    public final static int LEFT_BACK_TURN_ID = 31;
    public final static int RIGHT_FRONT_TURN_ID = 21;
    public final static int RIGHT_BACK_TURN_ID = 41;

    public final static int LEFT_FRONT_CANCODER_ID = 12;
    public final static int LEFT_BACK_CANCODER_ID = 32;
    public final static int RIGHT_FRONT_CANCODER_ID = 22;
    public final static int RIGHT_BACK_CANCODER_ID = 42;

    public final static int LEFT_FRONT_CANCODER_ID_M_TWO = 13;
    public final static int LEFT_BACK_CANCODER_ID_M_TWO = 33;
    public final static int RIGHT_FRONT_CANCODER_ID_M_TWO = 23;
    public final static int RIGHT_BACK_CANCODER_ID_M_TWO = 43;

    public final static int LEFT_FRONT_CANCODER_ID_M_THREE = 14;
    public final static int LEFT_BACK_CANCODER_ID_M_THREE = 34;
    public final static int RIGHT_FRONT_CANCODER_ID_M_THREE = 24;
    public final static int RIGHT_BACK_CANCODER_ID_M_THREE = 44;

    // TODO
    public final static int ENCODER_TICKS_PER_ROTATION = 4096; // 2048 NEED NUMBER
    public final static double WHEEL_DIAMETER = 4.0;// inches

    public final static boolean FIELD_BASED = false;

    public final static int HUNDRED_MS_IN_SEC = 10;

    // TODO: get true constants
    // public final static int JOYSTICK_LEFT_X_AXIS = 0;
    // public final static int JOYSTICK_LEFT_Y_AXIS = 1;
    // public final static int JOYSTICK_RIGHT_X_AXIS = 2;
    // public final static int JOYSTICK_RIGHT_Y_AXIS = 3;

    public final static int JOYSTICK_X_AXIS = 0;
    public final static int JOYSTICK_Y_AXIS = 1;
    public final static int JOYSTICK_Z_ROTATION = 2;
    // TODO: Math constants
    public final static double INCHES_2_METERS = 0.0254;

    public static int LEFT_FRONT_ANGLE_ENCODER_OFFSET = 4096 / 2;
    public static int LEFT_BACK_ANGLE_ENCODER_OFFSET = 4096 / 2;
    public static int RIGHT_FRONT_ANGLE_ENCODER_OFFSET = 0;
    public static int RIGHT_BACK_ANGLE_ENCODER_OFFSET = 4096 / 4;

    public final static double TRACK_WIDTH = (22.0 - 2.625 - 2.625) * INCHES_2_METERS; // number in meters
    public final static double WHEEL_BASE = (22.0 - 2.625 - 2.625) * INCHES_2_METERS; // number in meters

    public final static String SUBSYSTEM_VERSION = "Test";

    public final static int DRIVER_JOYSTICK_ID = 0;
    public final static int OPERATOR_JOYSTICK_ID = 1;
    public final static int AIRPLANE_JOYSTICK_ID = 4; // TODO: Subject to change

    // Button IDs
    // public static final int SQUARE = 1;
    // public static final int X_BUTTON = 2;
    // public static final int CIRCLE = 3;
    // public static final int TRIANGLE = 4;
    // public static final int LEFT_BUMPER = 5;
    // public static final int RIGHT_BUMPER = 6;
    // public static final int LEFT_TRIGGER = 7;
    // public static final int RIGHT_TRIGGER = 8;
    // public static final int SHARE = 9;
    // public static final int OPTIONS = 10;

    // Swerve Button IDs
    public static final int TRIGGER = 1;
    public static final int THUMB_BUTTON = 2;
    public static final int THREE_BUTTON = 3;
    public static final int FOUR_BUTTON = 4;
    public static final int FIVE_BUTTON = 5;
    public static final int SIX_BUTTON = 6;
    public static final int SEVEN_BUTTON = 7;
    public static final int EIGHT_BUTTON = 8;
    public static final int NINE_BUTTON = 9;
    public static final int TEN_BUTTON = 10;
    public static final int ELEVEN_BUTTON = 11;
    public static final int TWELVE_BUTTON = 12;

    public static final int CANCODER_TICS_PER_ROTATION = 4096;

    // TODO: Get the home offsets
    public static final int LEFT_FRONT_CANCODER_HOME = 0;
    public static final int LEFT_BACK_CANCODER_HOME = 0;
    public static final int RIGHT_FRONT_CANCODER_HOME = 0;
    public static final int RIGHT_BACK_CANCODER_HOME = 0;

    // TODO: Get zero -> let zero be unit circle zero
    public static final int LEFT_FRONT_CANCODER_ZERO = 0;
    public static final int LEFT_BACK_CANCODER_ZERO = 0;
    public static final int RIGHT_FRONT_CANCODER_ZERO = 0;
    public static final int RIGHT_BACK_CANCODER_ZERO = 0;

}
