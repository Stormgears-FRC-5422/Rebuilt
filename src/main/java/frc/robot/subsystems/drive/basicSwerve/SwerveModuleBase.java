package frc.robot.subsystems.drive.basicSwerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants;
import frc.robot.Options.DriveOptions;
import frc.robot.subsystems.drive.basicSwerve.config.ModuleConfig;
import frc.robot.subsystems.drive.basicSwerve.config.PidConfig;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

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

    public Angle steerOffset;

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
     * Set the module state (velocity and angle) with optimization.
     * @param state Target swerve module state
     */
    public void setSwerveModuleState(SwerveModuleState state) {
        if (Constants.Drive.angleOptimize) {
            state.optimize(new Rotation2d(getSteerAngle()));
        }
        if (Constants.Drive.cosineOptimize) {
            state.cosineScale(new Rotation2d(getSteerAngle()));
        }
        setDriveVelocity(MetersPerSecond.of(state.speedMetersPerSecond));
        setSteerAngle(Degrees.of(state.angle.getDegrees()));
    }

    /**
     * Get the current drive velocity
     * @return Current drive velocity
     */
    public abstract LinearVelocity getDriveVelocity();

    /**
     * Get the current steering angle
     * @return Current steer angle
     */
    public abstract Angle getSteerAngle();

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
        Logger.recordOutput("Mod " + name + "/driveVelocity", getDriveVelocity().in(MetersPerSecond));
        Logger.recordOutput("Mod " + name + "/steerAngle", getSteerAngle().in(Degrees));
    }

    /**
     * Console output with module name prefix
     * @param message Message to print
     */
    protected void console(String message) {
        System.out.println("SwerveModule (" + name + "): " + message);
    }

    // ==================== Unit Conversion Methods ====================
    // These are pure physics calculations based on module geometry.
    // Implementations use the appropriate method for their unit convention.

    /**
     * Convert wheel rotations per second to linear velocity in m/s.
     * Use when the encoder reports wheel-level rotational speed (e.g., TalonFX with SensorToMechanismRatio).
     */
    protected double wheelRPSToMetersPerSecond(double wheelRPS) {
        double circumference = Math.PI * config.wheelDiameter.in(Meters);
        return wheelRPS * circumference;
    }

    /**
     * Convert linear velocity in m/s to wheel rotations per second.
     * Use when setting wheel-level velocity setpoints (e.g., TalonFX with SensorToMechanismRatio).
     */
    protected double metersPerSecondToWheelRPS(double mps) {
        double circumference = Math.PI * config.wheelDiameter.in(Meters);
        return mps / circumference;
    }

    /**
     * Convert motor RPM to linear velocity in m/s (accounts for gear ratio).
     * Use when the encoder reports raw motor rotational speed (e.g., SparkMax).
     */
    protected double motorRPMToMetersPerSecond(double motorRPM) {
        double wheelRPM = motorRPM / config.driveRatio;
        double wheelRPS = wheelRPM / 60.0;
        return wheelRPSToMetersPerSecond(wheelRPS);
    }

    /**
     * Convert linear velocity in m/s to motor RPM (accounts for gear ratio).
     * Use when setting motor-level velocity setpoints (e.g., SparkMax).
     */
    protected double metersPerSecondToMotorRPM(double mps) {
        double wheelRPS = metersPerSecondToWheelRPS(mps);
        double wheelRPM = wheelRPS * 60.0;
        return wheelRPM * config.driveRatio;
    }

    /**
     * Convert motor rotations to steer angle in degrees (accounts for gear ratio).
     */
    protected double steerRotationsToDegrees(double rotations) {
        return rotations * 360.0 / config.steerRatio;
    }

    // ==================== CANcoder Configuration ====================

    /**
     * Configure a CANcoder with zero magnetic offset.
     * Use this when the offset will be applied elsewhere (e.g., to a relative encoder).
     */
    protected StatusCode configureCancoderMagnetOffset(CANcoder cancoder) {
        return configureCancoderMagnetOffset(cancoder, 0.0);
    }

    /**
     * Configure a CANcoder with a specified magnetic offset.
     * Use this when the CANcoder is used directly for feedback (e.g., TalonFX remote sensor).
     * The offset is in rotations (CANcoder native units).
     */
    protected StatusCode configureCancoderMagnetOffset(CANcoder cancoder, double offsetRotations) {
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.MagnetOffset = offsetRotations;
        StatusCode status = cancoder.getConfigurator().apply(canCoderConfig, 0.050);
        if (status.isOK()) {
            console("Successfully configured CANcoder " + config.encoderID + " magnetic offset: " + offsetRotations + " rotations");
        } else {
            console("Failed to configure CANcoder " + config.encoderID + " magnetic offset: " + status);
        }
        return status;
    }
}
