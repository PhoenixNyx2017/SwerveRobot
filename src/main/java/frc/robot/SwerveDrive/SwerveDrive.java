// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SwerveDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CAN;
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

  CANCoder leftFrontCanCoder, leftBackCanCoder, rightFrontCanCoder, rightBackCanCoder;

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

    leftFrontCanCoder = new CANCoder(Constants.LEFT_FRONT_CANCODER_ID);
    leftBackCanCoder = new CANCoder(Constants.LEFT_BACK_CANCODER_ID);
    rightFrontCanCoder = new CANCoder(Constants.RIGHT_FRONT_CANCODER_ID);
    rightBackCanCoder = new CANCoder(Constants.RIGHT_BACK_CANCODER_ID);

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

  public void drive(double forward, double strafe, double rotation) {

    // numbers are values from [-1.0, 1.0]

    // may need to adjust
    double forward_adjusted = forward * Constants.MAX_LIN_SPEED;
    double strafe_adjusted = strafe * Constants.MAX_LIN_SPEED;
    double rotation_adjusted = rotation * Constants.MAX_ROT_SPEED;

    // sClass.driveSwerve(forward_adjusted, strafe_adjusted, rotation_adjusted);
    sClass.driveSwerveKinematics(forward, strafe, rotation);

  }

  public void resetEncoders() {
    sClass.resetEncoders();
  }

  // ------------Helper Functions--------------------------

  public TalonFX getLeftFrontSteer() {
    return leftFrontSteer;
  }

  public TalonFX getLeftBackSteer() {
    return leftBackSteer;
  }

  public TalonFX getRightFrontSteer() {
    return rightFrontSteer;
  }

  public TalonFX getRightBackSteer() {
    return rightBackSteer;
  }

  // ---------returns Cancoders
  public CANCoder getLFCan() {
    return leftFrontCanCoder;
  }

  public CANCoder getLBCan() {
    return leftBackCanCoder;
  }

  public CANCoder getRFCan() {
    return rightFrontCanCoder;
  }

  public CANCoder getRBCan() {
    return rightBackCanCoder;
  }
}
