// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SwerveOdometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.SwerveClass;

/** Add your docs here. */
public class SwerveOdometry {

    // This will be used in encoder ticks yay
    int prev_left_front_drive_tic, prev_left_back_drive_tic;
    int prev_right_front_drive_tic, prev_right_back_drive_tic;
    int prev_left_front_turn_tic, prev_left_back_turn_tic;
    int prev_right_front_turn_tic, prev_right_back_turn_tic;

    SwerveClass sDrive;

    double prev_time;
    Rotation2d prev_gyro_angle;
    Pose2d pose; // pose stored in the odometry, will get updated as update is called.

    SwerveOdometry(SwerveClass sDrive, Rotation2d gyroAngle) {
        this.sDrive = sDrive;

    }

    public Pose2d getPose() {
        // Pose2d pose = new Pose2d();

        return pose;
    }

    public void update(Rotation2d gyro) {
        // TODO: update the pose
    }

    public void updateWithTime() {
        // TODO: Updates the pose with specific time

    }

    public void resetPosition(Pose2d position, Rotation2d gyroAngle) {
        // TODO: Resets the position on the field

    }

}
