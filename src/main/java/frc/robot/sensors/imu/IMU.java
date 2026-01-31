package frc.robot.sensors.imu;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Interface for IMU (Inertial Measurement Unit) operations.
 * Abstracts gyroscope functionality for different hardware implementations.
 */
public interface IMU {
    /**
     * Get the current rotation of the robot.
     * Implementations should apply any necessary inversion based on configuration.
     * @return Robot rotation as Rotation2d
     */
    Rotation2d getRotation();

    /**
     * Reset/zero the gyro orientation
     */
    void reset();

    /**
     * Set the gyro yaw to a specific angle in degrees.
     * @param angleDegrees The target yaw angle in degrees
     */
    void setYaw(double angleDegrees);

    /**
     * Update simulated gyro angle.
     * @param deltaAngleRadians Change in angle since last update
     */
    void updateSimulation(double deltaAngleRadians);
}
