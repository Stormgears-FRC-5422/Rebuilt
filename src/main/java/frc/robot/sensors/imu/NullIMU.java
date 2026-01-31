package frc.robot.sensors.imu;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Null object implementation of IMU.
 * Returns zero rotation - useful when no physical IMU is available.
 */
public class NullIMU implements IMU {
    @Override
    public Rotation2d getRotation() {
        return new Rotation2d();
    }

    @Override
    public void reset() {
        // No-op
    }

    @Override
    public void setYaw(double angleDegrees) {
        // No-op
    }

    @Override
    public void updateSimulation(double deltaAngleRadians) {
        // No-op
    }
}
