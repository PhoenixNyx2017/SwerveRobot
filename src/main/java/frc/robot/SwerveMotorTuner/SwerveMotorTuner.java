// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SwerveMotorTuner;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveMotorTuner extends SubsystemBase {
  /** Creates a new SwerveMotorTuner. */

  // used for tuning the encoders of the angle motors
  TalonFX leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;

  CANCoder leftFrontSteerCAN, leftBackSteerCAN, rightFrontSteerCAN, rightBackSteerCAN;

  int leftFrontEncoderTic = 0;
  int leftBackEncoderTic = 0;
  int rightFrontEncoderTic = 0;
  int rightBackEncoderTic = 0;

  public SwerveMotorTuner(TalonFX leftFront, TalonFX leftBack, TalonFX rightFront, TalonFX rightBack,
      CANCoder leftFrontCAN, CANCoder leftBackCAN, CANCoder rightFrontCAN, CANCoder rightBackCAN) {
    this.leftFrontMotor = leftFront;
    this.leftBackMotor = leftBack;
    this.rightFrontMotor = rightFront;
    this.rightBackMotor = rightBack;

    this.leftFrontSteerCAN = leftFrontCAN;
    this.leftBackSteerCAN = leftBackCAN;
    this.rightFrontSteerCAN = rightFrontCAN;
    this.rightBackSteerCAN = rightBackCAN;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void displayEncoderTic() {
    // displays the encoders on the smartDashboard
    SmartDashboard.putNumber("leftFront Encoder", leftFrontSteerCAN.getAbsolutePosition());
    SmartDashboard.putNumber("leftBack Encoder", leftBackSteerCAN.getAbsolutePosition());
    SmartDashboard.putNumber("rightFront Encoder", rightFrontSteerCAN.getAbsolutePosition());
    SmartDashboard.putNumber("rightBack Encoder", rightBackSteerCAN.getAbsolutePosition());

  }

  public void displayCanCoderTic() {
    // displays the encoders on the smartDashboard
    SmartDashboard.putNumber("leftFront CANEncoder", leftFrontSteerCAN.getAbsolutePosition());
    SmartDashboard.putNumber("leftBack CANEncoder", leftBackSteerCAN.getAbsolutePosition());
    SmartDashboard.putNumber("rightFront CANEncoder", rightFrontSteerCAN.getAbsolutePosition());
    SmartDashboard.putNumber("rightBack CANEncoder", rightBackSteerCAN.getAbsolutePosition());

    SmartDashboard.putNumber("leftFront CANEncoder (Position)", leftFrontSteerCAN.getPosition());
    SmartDashboard.putNumber("leftBack CANEncoder (Position)", leftBackSteerCAN.getPosition());
    SmartDashboard.putNumber("rightFront CANEncoder (Position)", rightFrontSteerCAN.getPosition());
    SmartDashboard.putNumber("rightBack CANEncoder (Position)", rightBackSteerCAN.getPosition());

    SmartDashboard.putNumber("leftFront Encoder (integrated)", leftFrontMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("leftBack Encoder (integrated)", leftBackMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("rightFront Encoder (integrated)", rightFrontMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("rightBack Encoder (integrated)", rightBackMotor.getSelectedSensorPosition());

    // leftFrontMotor.configSelectedFeedbackSensor();
    // leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);

  }

  public void set2Home() {
    // Wheels are set to 'home state'
    /*
    
    
    */

    // leftFrontSteerCAN.setPosition(Constants.LEFT_FRONT_CANCODER_HOME);
    // leftBackSteerCAN.setPosition(Constants.LEFT_BACK_CANCODER_HOME);
    // rightFrontSteerCAN.setPosition(Constants.RIGHT_BACK_CANCODER_HOME);
    // rightBackSteerCAN.setPosition(Constants.RIGHT_FRONT_CANCODER_HOME);

  }

  public void set2Zero() {
    // sets encoders to zero
    // leftFrontEncoderTic = 0;
    // leftBackEncoderTic = 0;
    // rightFrontEncoderTic = 0;
    // rightBackEncoderTic = 0;
    // leftFrontSteerCAN.setPosition(0);
    // leftBackSteerCAN.setPosition(0);
    // rightFrontSteerCAN.setPosition(0);
    // rightBackSteerCAN.setPosition(0);

    // leftFrontSteerCAN.setPosition(Constants.LEFT_FRONT_CANCODER_HOME);
    // leftBackSteerCAN.setPosition(Constants.LEFT_BACK_CANCODER_HOME));
    // rightFrontSteerCAN.setPosition(Constants.RIGHT_BACK_CANCODER_HOME);
    // rightBackSteerCAN.setPosition(Constants.RIGHT_FRONT_CANCODER_HOME);

  }

  public void increaseEncoderTic() {
    // increases the encoder tic
    // leftFrontEncoderTic++;
    // leftBackEncoderTic++;
    // rightFrontEncoderTic++;
    // rightBackEncoderTic++;

    // if (leftFrontEncoderTic > 2048) {
    // set2Zero();
    // }
  }

  public void decreaseEncoderTic() {
    // decreases the encoder tics
    // leftFrontEncoderTic--;
    // leftBackEncoderTic--;
    // rightFrontEncoderTic--;
    // rightBackEncoderTic--;

    // if (leftFrontEncoderTic < 0) {
    // set2Zero();
    // }
  }

  public void setMotors() {
    // sets the motors to
    // leftFrontMotor.set(ControlMode.Position, leftFrontEncoderTic);
    // leftBackMotor.set(ControlMode.Position, leftBackEncoderTic);
    // rightFrontMotor.set(ControlMode.Position, rightFrontEncoderTic);
    // rightBackMotor.set(ControlMode.Position, rightBackEncoderTic);
  }

}
