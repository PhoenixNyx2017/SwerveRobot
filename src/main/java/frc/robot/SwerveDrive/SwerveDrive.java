// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SwerveDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveClass;

public class SwerveDrive extends SubsystemBase {
  /** Creates a new SwerveDrive. */

  TalonFX leftFrontDrive, leftFrontSteer;
  TalonFX leftBackDrive, leftBackSteer;
  TalonFX rightFrontDrive, rightFrontSteer;
  TalonFX rightBackDrive, rightBackSteer;

  public SwerveClass sClass;

  AHRS imu;

  public SwerveDrive() {

    leftFrontDrive = new TalonFX(Constants.LEFT_FRONT_DRIVE_ID);
    leftBackDrive = new TalonFX(Constants.LEFT_BACK_DRIVE_ID);
    rightFrontDrive = new TalonFX(Constants.RIGHT_FRONT_DRIVE_ID);
    rightBackDrive = new TalonFX(Constants.RIGHT_BACK_DRIVE_ID);

    leftFrontSteer = new TalonFX(Constants.LEFT_FRONT_TURN_ID);
    leftBackSteer = new TalonFX(Constants.LEFT_BACK_TURN_ID);
    rightFrontSteer = new TalonFX(Constants.RIGHT_FRONT_TURN_ID);
    rightBackSteer = new TalonFX(Constants.RIGHT_BACK_TURN_ID);

    sClass = new SwerveClass(leftFrontDrive, leftFrontSteer,
        leftBackDrive, leftBackSteer,
        rightFrontDrive, rightFrontSteer,
        rightBackDrive, rightBackSteer);

    imu = new AHRS(SPI.Port.kMXP); // On board Gyro

    resetEncoders();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(SwerveClass sDrive, Joystick driver) {

    // numbers are values from [-1.0, 1.0]
    double forward = driver.getRawAxis(Constants.JOYSTICK_LEFT_Y_AXIS);
    double strafe = driver.getRawAxis(Constants.JOYSTICK_LEFT_X_AXIS);
    double rotation = driver.getRawAxis(Constants.JOYSTICK_RIGHT_X_AXIS);

    // may need to adjust

    sDrive.driveSwerve(forward, strafe, rotation);

  }

  public void resetEncoders() {
    sClass.resetEncoders();
  }
}
