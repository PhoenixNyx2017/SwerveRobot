// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SwerveMotorTuner.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SwerveMotorTuner.SwerveMotorTuner;

public class DisplayCanCoderValues extends CommandBase {
  /** Creates a new DisplayCanCoderValues. */
  SwerveMotorTuner sMotorTuner;

  public DisplayCanCoderValues(SwerveMotorTuner sMotorTuner) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.sMotorTuner = sMotorTuner;

    addRequirements(sMotorTuner);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sMotorTuner.displayCanCoderTic();
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
