// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.Math;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

/** Add your docs here. */
// This is a custom class for Swerve Drive
public class SwerveClass {

    public TalonFX leftFrontDriver, leftFrontTurner;
    public TalonFX leftBackDriver, leftBackTurner;
    public TalonFX rightFrontDriver, rightFrontTurner;
    public TalonFX rightBackDriver, rightBackTurner;

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
    }

    public void driveSwerve(double forward, double strafe, double rotation) {

        // forward - unit/sec
        // strafe - unit/sec
        // rotation - rad/sec

        double[] leftFrontVals = findLeftFront(forward, strafe, rotation);
        double[] leftBackVals = findLeftBack(forward, strafe, rotation);
        double[] rightFrontVals = findRightFront(forward, strafe, rotation);
        double[] rightBackVals = findRightBack(forward, strafe, rotation);

        // setting motor values
        // motors will be in speed units, Controlmode.velocity takes in native sensor
        // units/ 100ms

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
            rotAngle = Math.atan2(Constants.WHEEL_BASE / 2, Constants.TRACK_WIDTH / 2) - (Math.PI / 2);
        } else {
            rotAngle = Math.atan2(Constants.WHEEL_BASE / 2, Constants.TRACK_WIDTH / 2) + (Math.PI / 2);
        }

        double xComp = (linSpeedMag * Math.cos(linAngle)) + (rotSpeedMag * Math.cos(rotAngle));
        double yComp = (linSpeedMag * Math.sin(linAngle)) + (rotSpeedMag * Math.sin(rotAngle));

        leftFrontMotorVal[0] = Math.sqrt(Math.pow(xComp, 2) + Math.pow(xComp, 2)); // magnitude of the resulting vector
        leftFrontMotorVal[1] = Math.atan2(yComp, xComp); // angle of the vector

        return leftFrontMotorVal;
    }

    private double[] findLeftBack(double forward, double strafe, double rotation) {

        double[] leftBackMotorVal = { 0.0, 0.0 };

        double linSpeedMag = Math.sqrt(Math.pow(forward, 2) + Math.pow(strafe, 2));

        return leftBackMotorVal;
    }

    private double[] findRightFront(double forward, double strafe, double rotation) {

        double[] rightFrontMotorVal = { 0.0, 0.0 };
        return rightFrontMotorVal;
    }

    private double[] findRightBack(double forward, double strafe, double rotation) {

        double[] rightBackMotorVal = { 0.0, 0.0 };
        return rightBackMotorVal;
    }

}
