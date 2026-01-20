package frc.robot.subsystems.drive.basicSwerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
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
import static edu.wpi.first.units.Units.Radians;

/**
 * Abstract base class for swerve modules.
 * Provides common interface and utility methods for vendor-specific implementations.
 *
 * ORGANIZATION:
 * - Fields (configuration and state)
 * - Constructor (runs once at startup)
 * - Periodic (runs every 20ms)
 * - Control methods (called by commands)
 * - Sensor methods (queried by odometry)
 * - Diagnostic methods (for testing)
 * - Utility methods (helpers)
 */
public abstract class SwerveModuleBase {

    // ==================== CONTROL STATE ENUM ====================

    /**
     * Control state for module.
     * - setDriveVelocity/setSteerAngle set state to PID_CONTROL
     * - driveMotorSet/steerMotorSet set state to OPEN_LOOP
     * - stop() sets state to STOP
     * The periodic() method reads the state and sends appropriate commands to hardware.
     */
    protected enum ControlState {
        STOP,        // Motors stopped
        PID_CONTROL, // Closed-loop control (BasicSwerve)
        OPEN_LOOP    // Direct percent output (DiagnosticSwerve)
    }

    // ==================== FIELDS ====================

    public final int index;
    public final ModuleConfig config;
    public final String name;
    protected final PidConfig drivePidConfig;
    protected final PidConfig steerPidConfig;
    protected final DriveOptions options;
    public Angle steerOffset;

    // Control state (single state for entire module, implied by which control method was last called)
    protected ControlState controlState = ControlState.STOP;

    // ==================== CONSTRUCTOR (Runs Once at Startup) ====================

    public SwerveModuleBase(int index, ModuleConfig config, PidConfig drivePidConfig, PidConfig steerPidConfig) {
        this.index = index;
        this.config = config;
        this.drivePidConfig = drivePidConfig;
        this.steerPidConfig = steerPidConfig;
        this.name = config.name;
        this.steerOffset = config.encoderOffset;
        this.options = DriveOptions.create();
    }

    // ==================== PERIODIC (Runs Every 20ms) ====================

    /**
     * Periodic update - called automatically every robot loop (50 Hz)
     * Implementations should:
     * 1. Read sensor values
     * 2. Send motor control signals
     * 3. Update telemetry
     */
    public abstract void periodic();

    // ==================== CONTROL METHODS (Called by Commands) ====================

    /**
     * Set module targets using PID control (for BasicSwerve).
     * Sets both drive velocity and steer angle atomically.
     * @param driveSpeed Target drive velocity
     * @param steerAngle Target steering angle
     */
    public abstract void setTargets(LinearVelocity driveSpeed, Angle steerAngle);

    /**
     * Set module motors using open-loop control (for DiagnosticSwerve).
     * Sets both drive and steer percent outputs atomically.
     * @param drivePercent Drive motor percent output (-1.0 to 1.0)
     * @param steerPercent Steer motor percent output (-1.0 to 1.0)
     */
    public abstract void setMotors(double drivePercent, double steerPercent);

    /**
     * Set the module state (velocity and angle) with optimization.
     * This is the primary interface used by swerve drive kinematics.
     * @param state Target swerve module state
     */
    public void setSwerveModuleState(SwerveModuleState state) {
        if (Constants.Drive.angleOptimize) {
            state.optimize(new Rotation2d(getSteerAngle()));
        }
        if (Constants.Drive.cosineOptimize) {
            state.cosineScale(new Rotation2d(getSteerAngle()));
        }
        setTargets(
            MetersPerSecond.of(state.speedMetersPerSecond),
            Degrees.of(state.angle.getDegrees())
        );
    }

    /**
     * Stop the module (both drive and steer motors)
     */
    public abstract void stop();

    // ==================== SENSOR METHODS (Queried by Odometry) ====================

    /**
     * Get the current drive velocity
     * @return Current drive velocity in m/s
     */
    public abstract LinearVelocity getDriveVelocity();

    /**
     * Get the current steering angle
     * @return Current steer angle (module-relative, 0° = forward)
     */
    public abstract Angle getSteerAngle();

    /**
     * Get the cumulative distance traveled by the drive motor
     * @return Distance in meters
     */
    public abstract double getDrivePositionMeters();

    /**
     * Get the module's current position (distance + angle) for odometry
     * @return SwerveModulePosition with distance traveled and current angle
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            getDrivePositionMeters(),
            new Rotation2d(getSteerAngle().in(Radians))
        );
    }

    /**
     * Get the maximum linear velocity this module can achieve
     * @return Maximum linear velocity in m/s
     */
    public abstract LinearVelocity getMaxLinearVelocity();

    // ==================== DIAGNOSTIC & TESTING METHODS ====================

    /**
     * Log telemetry data for this module using hardware-agnostic methods.
     * All values are accessed through abstract methods, so this works with any hardware.
     */
    protected void telemetry() {
        String basePath = "Drive/Modules/" + name;
        Logger.recordOutput(basePath + "/Drive/Velocity", getDriveVelocity().in(MetersPerSecond));
        Logger.recordOutput(basePath + "/Drive/Position", getDrivePositionMeters());
        Logger.recordOutput(basePath + "/Steer/Angle", getSteerAngle().in(Degrees));
    }

    /**
     * Console output with module name prefix
     * @param message Message to print
     */
    protected void console(String message) {
        System.out.println("SwerveModule (" + name + "): " + message);
    }

    // ==================== UTILITY METHODS ====================

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

    // ==================== UNIT CONVERSION METHODS ====================
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

    // ==================== CANCODER CONFIGURATION ====================

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
