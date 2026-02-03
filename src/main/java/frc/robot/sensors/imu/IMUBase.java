package frc.robot.sensors.imu;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import static edu.wpi.first.units.Units.Degrees;

/**
 * Abstract base class for IMU implementations.
 * Handles common functionality like inversion and console logging.
 */
public abstract class IMUBase implements IMU {
    protected final String name;
    protected final double invertMultiplier;
    protected final boolean isSimulation;

    protected IMUBase(String name) {
        this.name = name;
        this.invertMultiplier = Constants.Imu.inverted ? 1.0 : -1.0;
        this.isSimulation = RobotBase.isSimulation();
    }

    /**
     * Get the raw angle from the hardware in degrees.
     * Implementations should return the native value without any inversion.
     */
    protected abstract double getRawAngle();

    @Override
    public Rotation2d getRotation() {
        return new Rotation2d(Degrees.of(invertMultiplier * getRawAngle()));
    }

    /**
     * Update simulated gyro angle. Override in subclasses that support simulation.
     * @param deltaAngleRadians Change in angle since last update
     */
    public void updateSimulation(double deltaAngleRadians) {
        // Default: no-op (for IMUs that don't support simulation)
    }

    protected void console(String message) {
        System.out.println("IMU (" + name + "): " + message);
    }
}
