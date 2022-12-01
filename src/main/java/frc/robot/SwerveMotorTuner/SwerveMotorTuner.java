// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SwerveMotorTuner;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveMotorTuner extends SubsystemBase {
  /** Creates a new SwerveMotorTuner. */

  // used for tuning the encoders of the angle motors
  TalonFX leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;

  int leftFrontEncoderTic = 0;
  int leftBackEncoderTic = 0;
  int rightFrontEncoderTic = 0;
  int rightBackEncoderTic = 0;

  public SwerveMotorTuner(TalonFX leftFront, TalonFX leftBack, TalonFX rightFront, TalonFX rightBack) {
    this.leftFrontMotor = leftFront;
    this.leftBackMotor = leftBack;
    this.rightFrontMotor = rightFront;
    this.rightBackMotor = rightBack;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void displayEncoderTic() {
    // displays the encoders on the smartDashboard
    SmartDashboard.putNumber("leftFront Encoder", leftFrontEncoderTic);
    SmartDashboard.putNumber("leftBack Encoder", leftBackEncoderTic);
    SmartDashboard.putNumber("rightFront Encoder", rightFrontEncoderTic);
    SmartDashboard.putNumber("rightBack Encoder", rightBackEncoderTic);

  }

  public void set2Zero() {
    // sets encoders to zero
    leftFrontEncoderTic = 0;
    leftBackEncoderTic = 0;
    rightFrontEncoderTic = 0;
    rightBackEncoderTic = 0;

  }

  public void increaseEncoderTic() {
    // increases the encoder tic
    leftFrontEncoderTic++;
    leftBackEncoderTic++;
    rightFrontEncoderTic++;
    rightBackEncoderTic++;

    if (leftFrontEncoderTic > 2048) {
      set2Zero();
    }
  }

  public void decreaseEncoderTic() {
    // decreases the encoder tics
    leftFrontEncoderTic--;
    leftBackEncoderTic--;
    rightFrontEncoderTic--;
    rightBackEncoderTic--;

    if (leftFrontEncoderTic < 0) {
      set2Zero();
    }
  }

  public void setMotors() {
    // sets the motors to
    leftFrontMotor.set(ControlMode.Position, leftFrontEncoderTic);
    leftBackMotor.set(ControlMode.Position, leftBackEncoderTic);
    rightFrontMotor.set(ControlMode.Position, rightFrontEncoderTic);
    rightBackMotor.set(ControlMode.Position, rightBackEncoderTic);
  }

}
