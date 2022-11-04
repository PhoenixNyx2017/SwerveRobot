// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SwerveDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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

  SwerveClass sDrive;

  public SwerveDrive() {

    leftFrontDrive = new TalonFX(Constants.LEFT_FRONT_DRIVE_ID);
    leftBackDrive = new TalonFX(Constants.LEFT_BACK_DRIVE_ID);
    rightFrontDrive = new TalonFX(Constants.RIGHT_FRONT_DRIVE_ID);
    rightBackDrive = new TalonFX(Constants.RIGHT_BACK_DRIVE_ID);

    leftFrontSteer = new TalonFX(Constants.LEFT_FRONT_TURN_ID);
    leftBackSteer = new TalonFX(Constants.LEFT_BACK_TURN_ID);
    rightFrontSteer = new TalonFX(Constants.RIGHT_FRONT_TURN_ID);
    rightBackSteer = new TalonFX(Constants.RIGHT_BACK_TURN_ID);

    sDrive = new SwerveClass(leftFrontDrive, leftFrontSteer,
        leftBackDrive, leftBackSteer,
        rightFrontDrive, rightFrontSteer,
        rightBackDrive, rightBackSteer);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
