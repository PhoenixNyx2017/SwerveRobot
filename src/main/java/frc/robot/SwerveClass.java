// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.Math;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

/** Add your docs here. */
// This is a custom class for Swerve Drive
public class SwerveClass {

    public TalonFX leftFrontDriver, leftFrontTurner;
    public TalonFX leftBackDriver, leftBackTurner;
    public TalonFX rightFrontDriver, rightFrontTurner;
    public TalonFX rightBackDriver, rightBackTurner;

    // TODO: Add IMU here
    private AHRS imu;

    public SwerveClass(TalonFX leftFrontDriver, TalonFX leftFrontTurner,
            TalonFX leftBackDriver, TalonFX leftBackTurner,
            TalonFX rightFrontDriver, TalonFX rightFrontTurner,
            TalonFX rightBackDriver, TalonFX rightBackTurner) {

        this.leftFrontDriver = leftFrontDriver;
        this.leftFrontTurner = leftFrontTurner;
        this.leftBackDriver = leftBackDriver;
        this.leftBackTurner = leftBackTurner;

        this.rightFrontDriver = rightFrontDriver;
        this.rightFrontTurner = rightFrontTurner;
        this.rightBackDriver = rightFrontDriver;
        this.rightBackTurner = rightBackTurner;

        // imu = new AHRS(SPI.Port.kMXP);
    }

    public void driveSwerve(double forward, double strafe, double rotation) {

        // forward - unit/sec
        // strafe - unit/sec
        // rotation - rad/sec

        // if (Constants.FIELD_BASED) {
        // // TODO: if the values are field based, adjust forward and strafe values to
        // be
        // // robot based.

        // }

        // Robot based
        double[] leftFrontVals = findLeftFront(forward, strafe, rotation);
        double[] leftBackVals = findLeftBack(forward, strafe, rotation);
        double[] rightFrontVals = findRightFront(forward, strafe, rotation);
        double[] rightBackVals = findRightBack(forward, strafe, rotation);

        // setting motor values
        // motors will be in speed units, Controlmode.velocity takes in native sensor
        // units/ 100ms

        // TODO: translate values into (output value is in encoder ticks or an analog
        // value, dependant on sensor)

        // setting
        leftFrontDriver.set(ControlMode.Velocity,
                (int) ((leftFrontVals[0]
                        * (Constants.ENCODER_TICKS_PER_ROTATION / (Constants.WHEEL_DIAMETER * Math.PI)))
                        / Constants.HUNDRED_MS_IN_SEC));
        leftBackDriver.set(ControlMode.Velocity,
                (int) ((leftBackVals[0]
                        * (Constants.ENCODER_TICKS_PER_ROTATION / (Constants.WHEEL_DIAMETER * Math.PI)))
                        / Constants.HUNDRED_MS_IN_SEC));
        rightFrontDriver.set(ControlMode.Velocity,
                (int) ((rightFrontVals[0]
                        * (Constants.ENCODER_TICKS_PER_ROTATION / (Constants.WHEEL_DIAMETER * Math.PI)))
                        / Constants.HUNDRED_MS_IN_SEC));
        rightBackDriver.set(ControlMode.Velocity,
                (int) ((rightBackVals[0]
                        * (Constants.ENCODER_TICKS_PER_ROTATION / (Constants.WHEEL_DIAMETER * Math.PI)))
                        / Constants.HUNDRED_MS_IN_SEC));

        leftFrontTurner.set(ControlMode.Position,
                (int) (leftFrontVals[1] / (2 * Math.PI)) * Constants.ENCODER_TICKS_PER_ROTATION);
        leftBackTurner.set(ControlMode.Position,
                (int) (leftBackVals[1] / (2 * Math.PI)) * Constants.ENCODER_TICKS_PER_ROTATION);
        rightFrontTurner.set(ControlMode.Position,
                (int) (rightFrontVals[1] / (2 * Math.PI)) * Constants.ENCODER_TICKS_PER_ROTATION);
        rightBackTurner.set(ControlMode.Position,
                (int) (rightBackVals[1] / (2 * Math.PI)) * Constants.ENCODER_TICKS_PER_ROTATION);

    }

    public void driveSwerveVerTwo(double forward, double strafe, double rotation) {
        double radius = Math.sqrt(Math.pow(Constants.WHEEL_BASE, 2) + Math.pow(Constants.TRACK_WIDTH, 2));

        double a = strafe - rotation * (Constants.WHEEL_BASE / radius);
        double b = strafe + rotation * (Constants.WHEEL_BASE / radius);
        double c = forward - rotation * (Constants.TRACK_WIDTH / radius);
        double d = forward + rotation * (Constants.TRACK_WIDTH / radius);

        double ws1 = Math.sqrt(Math.pow(b, 2) + Math.pow(c, 2));
        double ws2 = Math.sqrt(Math.pow(b, 2) + Math.pow(d, 2));
        double ws3 = Math.sqrt(Math.pow(a, 2) + Math.pow(d, 2));
        double ws4 = Math.sqrt(Math.pow(a, 2) + Math.pow(c, 2));

        double wa1 = Math.atan2(b, c) * (180.0 / Math.PI);
        double wa2 = Math.atan2(b, d) * (180.0 / Math.PI);
        double wa3 = Math.atan2(a, d) * (180.0 / Math.PI);
        double wa4 = Math.atan2(a, c) * (180.0 / Math.PI);
    }

    public void resetEncoders() {

        // sets all the encoder values to zero
        this.leftBackDriver.setSelectedSensorPosition(0, 0, 0);
        this.leftFrontDriver.setSelectedSensorPosition(0, 0, 0);
        this.rightFrontDriver.setSelectedSensorPosition(0, 0, 0);
        this.rightBackDriver.setSelectedSensorPosition(0, 0, 0);

        this.leftFrontTurner.setSelectedSensorPosition(0, 0, 0);
        this.leftBackTurner.setSelectedSensorPosition(0, 0, 0);
        this.rightFrontTurner.setSelectedSensorPosition(0, 0, 0);
        this.rightBackTurner.setSelectedSensorPosition(0, 0, 0);
    }

    // ---------PRIVATE METHODS FOR THE MATH------------------------------------

    private double[] findLeftFront(double forward, double strafe, double rotation) {

        double[] leftFrontMotorVal = { 0.0, 0.0 }; // intialize the array

        // Linear Magnitude
        double linSpeedMag = Math.sqrt((forward * forward) + (strafe * strafe));
        // Rotational Magnitude
        double rotSpeedMag = rotation
                * Math.sqrt(Math.pow(Constants.WHEEL_BASE / 2, 2) + Math.pow(Constants.TRACK_WIDTH / 2, 2));

        double linAngle = Math.atan2(forward, strafe); // Linear Angle
        double rotAngle; // Rotation vector angle
        if (rotation < 0) {
            rotAngle = Math.atan2(Constants.WHEEL_BASE / 2, -Constants.TRACK_WIDTH / 2) - (Math.PI / 2);
        } else {
            rotAngle = Math.atan2(Constants.WHEEL_BASE / 2, -Constants.TRACK_WIDTH / 2) + (Math.PI / 2);
        }

        double xComp = (linSpeedMag * Math.cos(linAngle)) + (rotSpeedMag * Math.cos(rotAngle));
        double yComp = (linSpeedMag * Math.sin(linAngle)) + (rotSpeedMag * Math.sin(rotAngle));

        leftFrontMotorVal[0] = Math.sqrt(Math.pow(xComp, 2) + Math.pow(xComp, 2)); // magnitude of the resulting vector
        leftFrontMotorVal[1] = Math.atan2(yComp, xComp); // angle of the vector

        return leftFrontMotorVal;
    }

    private double[] findLeftBack(double forward, double strafe, double rotation) {

        double[] leftBackMotorVal = { 0.0, 0.0 };

        // Linear Magnitude
        double linSpeedMag = Math.sqrt((forward * forward) + (strafe * strafe));
        // Rotational Magnitude
        double rotSpeedMag = rotation
                * Math.sqrt(Math.pow(Constants.WHEEL_BASE / 2, 2) + Math.pow(Constants.TRACK_WIDTH / 2, 2));

        double linAngle = Math.atan2(forward, strafe); // Linear Angle
        double rotAngle; // Rotation vector angle
        if (rotation < 0) {
            rotAngle = Math.atan2(-Constants.WHEEL_BASE / 2, -Constants.TRACK_WIDTH / 2) - (Math.PI / 2);
        } else {
            rotAngle = Math.atan2(-Constants.WHEEL_BASE / 2, -Constants.TRACK_WIDTH / 2) + (Math.PI / 2);
        }

        double xComp = (linSpeedMag * Math.cos(linAngle)) + (rotSpeedMag * Math.cos(rotAngle));
        double yComp = (linSpeedMag * Math.sin(linAngle)) + (rotSpeedMag * Math.sin(rotAngle));

        leftBackMotorVal[0] = Math.sqrt(Math.pow(xComp, 2) + Math.pow(xComp, 2)); // magnitude of the resulting vector
        leftBackMotorVal[1] = Math.atan2(yComp, xComp);

        return leftBackMotorVal;
    }

    private double[] findRightFront(double forward, double strafe, double rotation) {

        double[] rightFrontMotorVal = { 0.0, 0.0 };

        // Linear Magnitude
        double linSpeedMag = Math.sqrt((forward * forward) + (strafe * strafe));
        // Rotational Magnitude
        double rotSpeedMag = rotation
                * Math.sqrt(Math.pow(Constants.WHEEL_BASE / 2, 2) + Math.pow(Constants.TRACK_WIDTH / 2, 2));

        double linAngle = Math.atan2(forward, strafe); // Linear Angle
        double rotAngle; // Rotation vector angle
        if (rotation < 0) {
            rotAngle = Math.atan2(Constants.WHEEL_BASE / 2, Constants.TRACK_WIDTH / 2) - (Math.PI / 2);
        } else {
            rotAngle = Math.atan2(Constants.WHEEL_BASE / 2, Constants.TRACK_WIDTH / 2) + (Math.PI / 2);
        }

        double xComp = (linSpeedMag * Math.cos(linAngle)) + (rotSpeedMag * Math.cos(rotAngle));
        double yComp = (linSpeedMag * Math.sin(linAngle)) + (rotSpeedMag * Math.sin(rotAngle));

        rightFrontMotorVal[0] = Math.sqrt(Math.pow(xComp, 2) + Math.pow(xComp, 2)); // magnitude of the resulting vector
        rightFrontMotorVal[1] = Math.atan2(yComp, xComp); // angle of the vector

        return rightFrontMotorVal;
    }

    private double[] findRightBack(double forward, double strafe, double rotation) {

        double[] rightBackMotorVal = { 0.0, 0.0 };

        // Linear Magnitude
        double linSpeedMag = Math.sqrt((forward * forward) + (strafe * strafe));
        // Rotational Magnitude
        double rotSpeedMag = rotation
                * Math.sqrt(Math.pow(Constants.WHEEL_BASE / 2, 2) + Math.pow(Constants.TRACK_WIDTH / 2, 2));

        double linAngle = Math.atan2(forward, strafe); // Linear Angle
        double rotAngle; // Rotation vector angle
        if (rotation < 0) {
            rotAngle = Math.atan2(-Constants.WHEEL_BASE / 2, Constants.TRACK_WIDTH / 2) - (Math.PI / 2);
        } else {
            rotAngle = Math.atan2(-Constants.WHEEL_BASE / 2, Constants.TRACK_WIDTH / 2) + (Math.PI / 2);
        }

        double xComp = (linSpeedMag * Math.cos(linAngle)) + (rotSpeedMag * Math.cos(rotAngle));
        double yComp = (linSpeedMag * Math.sin(linAngle)) + (rotSpeedMag * Math.sin(rotAngle));

        rightBackMotorVal[0] = Math.sqrt(Math.pow(xComp, 2) + Math.pow(xComp, 2)); // magnitude of the resulting vector
        rightBackMotorVal[1] = Math.atan2(yComp, xComp); // angle of the vector

        return rightBackMotorVal;
    }

}
