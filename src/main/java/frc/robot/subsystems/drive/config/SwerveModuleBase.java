package frc.robot.subsystems.drive.config;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Options.DriveOptions;
import org.littletonrobotics.junction.Logger;

/**
 * Abstract base class for swerve modules.
 * Provides common interface and utility methods for vendor-specific implementations.
 */
public abstract class SwerveModuleBase {

    public final int index;
    public final ModuleConfig config;
    public final String name;
    protected final PidConfig drivePidConfig;
    protected final PidConfig steerPidConfig;
    protected final DriveOptions options;

    public double steerOffset;

    public SwerveModuleBase(int index, ModuleConfig config, PidConfig drivePidConfig, PidConfig steerPidConfig) {
        this.index = index;
        this.config = config;
        this.drivePidConfig = drivePidConfig;
        this.steerPidConfig = steerPidConfig;
        this.name = config.name;
        this.steerOffset = config.encoderOffset;
        this.options = DriveOptions.create();
    }

    /**
     * Set the drive motor velocity
     * @param speed Target velocity
     */
    public abstract void setDriveVelocity(LinearVelocity speed);

    /**
     * Set the steering angle
     * @param angle Target angle
     */
    public abstract void setSteerAngle(Angle angle);

    /**
     * Set the module state (velocity and angle)
     * @param state Target swerve module state
     */
    public abstract void setSwerveModuleState(SwerveModuleState state);

    /**
     * Get the current drive velocity in m/s
     * @return Current drive velocity
     */
    public abstract double getDriveVelocity();

    /**
     * Get the current steering angle in degrees
     * @return Current steer angle
     */
    public abstract double getSteerAngle();

    /**
     * Get the maximum linear velocity this module can achieve
     * @return Maximum linear velocity
     */
    public abstract LinearVelocity getMaxLinearVelocity();

    /**
     * Periodic update - called every robot loop
     */
    public abstract void periodic();

    /**
     * Stop the module
     */
    public abstract void stop();

    /**
     * Set drive motor voltage directly (for diagnostic purposes)
     * @param voltage Voltage to apply (-1.0 to 1.0)
     */
    public abstract void driveMotorSet(double voltage);

    /**
     * Set steer motor voltage directly (for diagnostic purposes)
     * @param voltage Voltage to apply (-1.0 to 1.0)
     */
    public abstract void steerMotorSet(double voltage);

    /**
     * Normalize angle to -180 to +180 degree range
     * @param unwoundAngleDegrees Angle in degrees (any range)
     * @return Normalized angle in range [-180, 180]
     */
    protected double normalizeDegrees(double unwoundAngleDegrees) {
        // First, use modulo to get the angle within a 360 degree range (0 to 360)
        double angleMod = unwoundAngleDegrees % 360.0;

        // Handle negative angles (modulo in Java can return negative values)
        if (angleMod < 0) {
            angleMod += 360.0;
        }

        // Now convert from 0-360 range to -180 to 180 range
        if (angleMod > 180.0) {
            angleMod -= 360.0;
        }

        return angleMod;
    }

    /**
     * Log telemetry data for this module using hardware-agnostic methods.
     * All values are accessed through abstract methods, so this works with any hardware.
     */
    protected void telemetry() {
        Logger.recordOutput("Mod " + name + "/driveVelocity", getDriveVelocity());
        Logger.recordOutput("Mod " + name + "/steerAngle", getSteerAngle());
    }

    /**
     * Console output with module name prefix
     * @param message Message to print
     */
    protected void console(String message) {
        System.out.println("SwerveModule (" + name + "): " + message);
    }
}
