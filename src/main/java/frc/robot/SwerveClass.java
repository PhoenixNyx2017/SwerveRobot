// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.Math;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;

/** Add your docs here. */
// This is a custom class for Swerve Drive
public class SwerveClass {

    public TalonFX leftFrontDriver, leftFrontTurner;
    public TalonFX leftBackDriver, leftBackTurner;
    public TalonFX rightFrontDriver, rightFrontTurner;
    public TalonFX rightBackDriver, rightBackTurner;

    public SwerveDriveKinematics sDriveKinematics;
    public SwerveDriveOdometry sDriveOdometry;

    Translation2d leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel;
    // TODO: Add IMU here
    // private AHRS imu;
    public AHRS imu;

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

        imu = new AHRS(SPI.Port.kMXP);

        leftFrontWheel = new Translation2d(-Constants.WHEEL_BASE / 2, Constants.TRACK_WIDTH / 2);
        leftBackWheel = new Translation2d(-Constants.WHEEL_BASE / 2, -Constants.TRACK_WIDTH / 2);
        rightFrontWheel = new Translation2d(Constants.WHEEL_BASE / 2, Constants.TRACK_WIDTH / 2);
        rightBackWheel = new Translation2d(Constants.WHEEL_BASE / 2, -Constants.TRACK_WIDTH / 2);

        sDriveKinematics = new SwerveDriveKinematics(leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel);
        sDriveOdometry = new SwerveDriveOdometry(sDriveKinematics, imu.getRotation2d());

    }

    public void driveSwerve(double forward, double strafe, double rotation) {

        // forward - meters/sec
        // strafe - meters/sec
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

        // setting Motor values
        leftFrontDriver.set(ControlMode.Velocity, wheelSpeed2EncoderTics(leftFrontVals[0]));
        leftBackDriver.set(ControlMode.Velocity, wheelSpeed2EncoderTics(leftBackVals[0]));
        rightFrontDriver.set(ControlMode.Velocity, wheelSpeed2EncoderTics(rightFrontVals[0]));
        rightBackDriver.set(ControlMode.Velocity, wheelSpeed2EncoderTics(rightBackVals[0]));

        leftFrontTurner.set(ControlMode.Position, angle2Encoder(leftFrontVals[1]));
        leftBackTurner.set(ControlMode.Position, angle2Encoder(leftBackVals[1]));
        rightFrontTurner.set(ControlMode.Position, angle2Encoder(rightFrontVals[1]));
        rightBackTurner.set(ControlMode.Position, angle2Encoder(rightBackVals[1]));

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

    public void driveSwerveKinematics(double forward, double strafe, double rotation) {

        double xMetersPerSecond = forward * Constants.MAX_LIN_SPEED;
        double yMetersPerSecond = strafe * Constants.MAX_LIN_SPEED;
        double omegaRadiansPerSecond = rotation * Constants.MAX_ROT_SPEED;

        ChassisSpeeds inputs = new ChassisSpeeds(xMetersPerSecond, yMetersPerSecond, omegaRadiansPerSecond);

        SwerveModuleState[] modules = sDriveKinematics.toSwerveModuleStates(inputs);

        SwerveModuleState leftFrontModule = modules[0];
        SwerveModuleState leftBackModule = modules[1];
        SwerveModuleState rightFrontModule = modules[2];
        SwerveModuleState rightBackModule = modules[3];

        // TODO: get the encoder Ticks
        leftFrontDriver.set(ControlMode.Velocity, wheelSpeed2EncoderTics(leftFrontModule.speedMetersPerSecond));
        leftBackDriver.set(ControlMode.Velocity, wheelSpeed2EncoderTics(leftBackModule.speedMetersPerSecond));
        rightFrontDriver.set(ControlMode.Velocity, wheelSpeed2EncoderTics(rightFrontModule.speedMetersPerSecond));
        rightBackDriver.set(ControlMode.Velocity, wheelSpeed2EncoderTics(rightBackModule.speedMetersPerSecond));

        leftFrontTurner.set(ControlMode.Position,
                angle2Encoder(leftFrontModule.angle.getRadians()) + Constants.LEFT_FRONT_ANGLE_ENCODER_OFFSET);
        leftBackTurner.set(ControlMode.Position,
                angle2Encoder(leftBackModule.angle.getRadians()) + Constants.LEFT_BACK_ANGLE_ENCODER_OFFSET);
        rightFrontTurner.set(ControlMode.Position,
                angle2Encoder(rightFrontModule.angle.getRadians()) + Constants.RIGHT_FRONT_ANGLE_ENCODER_OFFSET);
        rightBackTurner.set(ControlMode.Position,
                angle2Encoder(rightBackModule.angle.getRadians()) + Constants.RIGHT_BACK_ANGLE_ENCODER_OFFSET);

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

    private int wheelSpeed2EncoderTics(double speed) {
        // meters/sec to encoder tics
        int encoders = (int) ((speed
                * (Constants.ENCODER_TICKS_PER_ROTATION
                        / (Constants.WHEEL_DIAMETER * Constants.INCHES_2_METERS * Math.PI)))
                / Constants.HUNDRED_MS_IN_SEC);

        return encoders;
    }

    private int angle2Encoder(double angle) {
        // radians to encoder tic
        int encoders = (int) ((angle / (2 * Math.PI)) * Constants.ENCODER_TICKS_PER_ROTATION);

        return encoders;

    }

}
