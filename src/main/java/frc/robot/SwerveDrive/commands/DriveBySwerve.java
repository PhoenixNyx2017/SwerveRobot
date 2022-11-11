// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SwerveDrive.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.SwerveDrive.SwerveDrive;

public class DriveBySwerve extends CommandBase {
  /** Creates a new DriveBySwerve. */

  SwerveDrive sDrive;
  Joystick driver;

  public DriveBySwerve(SwerveDrive sDrive, Joystick driver) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.sDrive = sDrive;
    this.driver = driver;

    addRequirements(sDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sDrive.drive(driver.getRawAxis(Constants.JOYSTICK_LEFT_Y_AXIS),
        driver.getRawAxis(Constants.JOYSTICK_LEFT_X_AXIS),
        driver.getRawAxis(Constants.JOYSTICK_RIGHT_X_AXIS));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
