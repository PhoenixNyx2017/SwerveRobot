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

    public static double MAX_LIN_SPEED = 5.0; // meters/sec
    public static double MAX_ROT_SPEED = 1.0; // rad/sec

    public final static double TRACK_WIDTH = 0.0; // number in meters
    public final static double WHEEL_BASE = 0.0; // number in meters

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

    public final static int ENCODER_TICKS_PER_ROTATION = 0; // NEED NUMBER
    public final static double WHEEL_DIAMETER = 0.0;// some UNIT

}
