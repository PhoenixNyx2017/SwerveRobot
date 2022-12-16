// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.SwerveDrive.SwerveDrive;
import frc.robot.SwerveDrive.commands.DriveBySwerve;
import frc.robot.SwerveDrive.commands.DriveMotorsSetSpeed;
import frc.robot.SwerveDrive.commands.DriveRightBackMotor;
import frc.robot.SwerveMotorTuner.SwerveMotorTuner;
import frc.robot.SwerveMotorTuner.commands.DecreaseEncoderTic;
import frc.robot.SwerveMotorTuner.commands.DisplayCanCoderValues;
import frc.robot.SwerveMotorTuner.commands.IncreaseEncoderTic;
import frc.robot.SwerveMotorTuner.commands.SetEncodersToZero;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  SwerveDrive sDrive;

  SwerveMotorTuner sMotorTuner;

  Joystick driver, operator, airplane;

  long startTime;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings

    sDrive = new SwerveDrive();
    configureButtonBindings();

    sDrive.setDefaultCommand(new DriveBySwerve(sDrive, airplane));

    sMotorTuner = new SwerveMotorTuner(sDrive.getLeftFrontSteer(), sDrive.getLeftBackSteer(),
        sDrive.getRightFrontSteer(), sDrive.getRightBackSteer(),
        sDrive.getLFCan(), sDrive.getLBCan(), sDrive.getRFCan(), sDrive.getRBCan());

    sMotorTuner.setDefaultCommand(new DisplayCanCoderValues(sMotorTuner));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    driver = new Joystick(Constants.DRIVER_JOYSTICK_ID);
    operator = new Joystick(Constants.OPERATOR_JOYSTICK_ID);
    airplane = new Joystick(Constants.AIRPLANE_JOYSTICK_ID);

    // new JoystickButton(operator, Constants.TRIANGLE).whenPressed(new
    // IncreaseEncoderTic(sMotorTuner));

    // new JoystickButton(operator, Constants.SQUARE).whenPressed(new
    // DecreaseEncoderTic(sMotorTuner));

    // new JoystickButton(operator, Constants.X_BUTTON).whenPressed(new
    // SetEncodersToZero(sMotorTuner));

    new JoystickButton(airplane, Constants.THUMB_BUTTON)
        .whileHeld(new DriveMotorsSetSpeed(sDrive));

    new JoystickButton(airplane, Constants.THREE_BUTTON)
        .whileHeld(new DriveRightBackMotor(sDrive));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  // // An ExampleCommand will run in autonomous

  // }
}
