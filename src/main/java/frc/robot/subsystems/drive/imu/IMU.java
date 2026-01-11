package frc.robot.subsystems.drive.imu;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Interface for IMU (Inertial Measurement Unit) operations.
 * Abstracts gyroscope functionality for different hardware implementations.
 */
public interface IMU {
    /**
     * Get the current rotation of the robot
     * @return Robot rotation as Rotation2d
     */
    Rotation2d getRotation();

    /**
     * Reset/zero the gyro orientation
     */
    void reset();

    /**
     * Get the yaw angle in degrees (for telemetry)
     * @return Yaw angle in degrees
     */
    double getYaw();
}
