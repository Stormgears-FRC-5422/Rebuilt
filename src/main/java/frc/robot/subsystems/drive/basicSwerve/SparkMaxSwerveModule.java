package frc.robot.subsystems.drive.basicSwerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants;
import frc.robot.subsystems.drive.basicSwerve.config.ModuleConfig;
import frc.utils.pid.PidConfig;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

/**
 * SparkMax-based swerve module implementation.
 * Uses REV Robotics SparkMax (NEO motors) and CTRE CANcoder for absolute position.
 *
 * ORGANIZATION:
 * - Fields (hardware and state)
 * - Constructor (runs once at startup)
 * - Configuration methods (called from constructor)
 * - Periodic (runs every 20ms - the main loop)
 * - Control methods (called by commands to SET targets)
 * - Sensor methods (queried to GET current state)
 * - Diagnostic methods (for testing)
 * - Utility methods (helpers)
 */
public class SparkMaxSwerveModule extends SwerveModuleBase {

    // ==================== HARDWARE ====================

    private final CANcoder steerCANCoder;
    private final SparkMax driveMotor;
    private final SparkMax steerMotor;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steerEncoder;
    private final SparkMaxConfig driveConfig;
    private final SparkMaxConfig steerConfig;
    private final SparkClosedLoopController steerController;
    private final SparkClosedLoopController driveController;

    // ==================== STATE ====================

    // Cached sensor values (updated in periodic - REV encoders are slow to query repeatedly)
    private double driveVelocity;
    private double steerAngle;

    // Target values (set by control methods, sent in periodic based on state)
    private double driveTarget = 0;  // m/s for PID_CONTROL, percent for OPEN_LOOP
    private double steerTarget = 0;  // Degrees for PID_CONTROL, percent for OPEN_LOOP

    // ==================== CONSTRUCTOR (Runs Once at Startup) ====================

    public SparkMaxSwerveModule(int index, ModuleConfig moduleConfig, PidConfig drivePidConfig, PidConfig steerPidConfig) {
        super(index, moduleConfig, drivePidConfig, steerPidConfig);

        // CANcoder setup
        steerCANCoder = new CANcoder(config.encoderID);
        configureCancoderMagnetOffset(steerCANCoder);  // Zero the CANcoder offset

        // Create hardware objects
        driveMotor = new SparkMax(config.driveID, SparkBase.MotorType.kBrushless);
        steerMotor = new SparkMax(config.steerID, SparkBase.MotorType.kBrushless);

        // Initialize config objects
        driveConfig = new SparkMaxConfig();
        steerConfig = new SparkMaxConfig();

        // Configure hardware
        configureDriveMotor();  // Set up drive PID, limits, etc.
        configureSteerMotor();  // Set up steer PID, limits, etc.

        // Cache controller and encoder references (for performance)
        driveController = driveMotor.getClosedLoopController();
        driveEncoder = driveMotor.getEncoder();
        steerController = steerMotor.getClosedLoopController();
        steerEncoder = steerMotor.getEncoder();

        // Sync steer encoder with CANcoder absolute position
        syncSteerEncoderWithCancoder();

        console("SparkMax module initialized");
    }

    // ==================== CONFIGURATION (Called from Constructor) ====================

    private void configureDriveMotor() {
        // Global motor configuration (current limits, voltage compensation)
        SparkMaxConfig globalConfig = new SparkMaxConfig();
        globalConfig.smartCurrentLimit(Constants.SparkConstants.CurrentLimit)
                    .voltageCompensation(Constants.SparkConstants.NominalVoltage);

        // Encoder configuration (convert units to m/s and meters)
        EncoderConfig driveEncoderConfig = new EncoderConfig();
        driveEncoderConfig.velocityConversionFactor(motorRPMToMetersPerSecond(1.0));  // RPM → m/s
        driveEncoderConfig.positionConversionFactor(motorRotationsToMeters(1.0));     // rotations → meters

        // Apply global and encoder config
        driveConfig.apply(globalConfig)
                   .openLoopRampRate(Constants.Drive.driveOpenLoopRampRate)
                   .inverted(config.driveInverted);
        driveConfig.apply(driveEncoderConfig);

        // Auto-calculate feedforward gain (kV is in volts per m/s)
        drivePidConfig.kV = 1.0 / getMaxLinearVelocity().in(MetersPerSecond);

        // PID configuration
        driveConfig.closedLoop
                .p(drivePidConfig.kP).i(drivePidConfig.kI).d(drivePidConfig.kD)
                .outputRange(drivePidConfig.kMin, drivePidConfig.kMax);
        driveConfig.closedLoop.feedForward.kV(drivePidConfig.kV);

        // Apply configuration with parameter persistence
        REVLibError error = driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        if (error == REVLibError.kOk) {
            console("Drive motor configured successfully");
        } else {
            console("WARNING: Drive motor configuration failed: " + error);
        }
    }

    private void configureSteerMotor() {
        // Global motor configuration (current limits, voltage compensation)
        SparkMaxConfig globalConfig = new SparkMaxConfig();
        globalConfig.smartCurrentLimit(Constants.SparkConstants.CurrentLimit)
                    .voltageCompensation(Constants.SparkConstants.NominalVoltage);

        // Encoder configuration (convert motor rotations to wheel degrees)
        // steerRotationsToDegrees() accounts for gear ratio (e.g., 150:1 → 150 rotations = 360°)
        EncoderConfig steerEncoderConfig = new EncoderConfig();
        steerEncoderConfig.positionConversionFactor(steerRotationsToDegrees(1.0));

        // Apply global and encoder config
        steerConfig.apply(globalConfig)
                   .openLoopRampRate(Constants.Drive.steerOpenLoopRampRate)
                   .inverted(config.steerInverted);
        steerConfig.apply(steerEncoderConfig);

        // PID configuration
        steerPidConfig.kV = 0;  // No feedforward for position control
        steerConfig.closedLoop
                .p(steerPidConfig.kP).i(steerPidConfig.kI).d(steerPidConfig.kD)
                .outputRange(steerPidConfig.kMin, steerPidConfig.kMax)
                .positionWrappingEnabled(true)        // Enable continuous rotation (0° = 360°)
                .positionWrappingInputRange(-180, 180);  // Define wrapping range
        steerConfig.closedLoop.feedForward.kV(steerPidConfig.kV);

        // Apply configuration with parameter persistence
        REVLibError error = steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        if (error == REVLibError.kOk) {
            console("Steer motor configured successfully");
        } else {
            console("WARNING: Steer motor configuration failed: " + error);
        }

        // Log configured PID gains for diagnostic purposes
        console("Steer PID - kP: " + steerPidConfig.kP +
                ", kI: " + steerPidConfig.kI +
                ", kD: " + steerPidConfig.kD);
    }

    private void syncSteerEncoderWithCancoder() {
        // Read absolute position from CANcoder
        double cancoderDegrees = steerCANCoder.getAbsolutePosition().getValue().in(Degrees);

        // Calculate module-relative wheel position (subtract offset)
        double wheelPositionDegrees = cancoderDegrees - steerOffset.in(Degrees);

        // Set the motor encoder position to match
        // After this, encoder position = 0° means module pointing at 0° (forward)
        steerEncoder.setPosition(wheelPositionDegrees);

        // Log sync information
        console("Synced steer encoder: CANcoder=" + cancoderDegrees +
                "° → wheel=" + wheelPositionDegrees + "°");
        console("Offset semantics: module angle 0° = CANcoder " + steerOffset.in(Degrees) + "°");
    }

    // ==================== PERIODIC (Runs Every 20ms - This is the Main Loop!) ====================

    @Override
    public void periodic() {
        // 1. READ SENSORS
        // Update cached sensor values (REV encoders are slow to query repeatedly)
        driveVelocity = driveEncoder.getVelocity();  // Already in m/s via conversion factor
        steerAngle = normalizeDegrees(steerEncoder.getPosition());  // Already in degrees via conversion factor

        // 2. SEND CONTROL SIGNALS BASED ON STATE
        // Control methods set the state and target; here we execute based on state

        switch (controlState) {
            case STOP:
                driveMotor.stopMotor();
                steerMotor.stopMotor();
                break;
            case PID_CONTROL:
                driveController.setSetpoint(driveTarget, SparkBase.ControlType.kVelocity);
                steerController.setSetpoint(steerTarget, SparkBase.ControlType.kPosition);
                break;
            case OPEN_LOOP:
                driveMotor.set(driveTarget);
                steerMotor.set(steerTarget);
                break;
        }

        // 3. LOG TELEMETRY
        telemetry();

        // 4. DYNAMIC RECONFIGURATION (if needed)
        // Check if PID gains or config have been changed at runtime
        if (steerPidConfig.isDirty()) {
            updatePidConfig(steerConfig, steerMotor, steerPidConfig, "Steer");
            steerPidConfig.clearDirty();
        }
        if (drivePidConfig.isDirty()) {
            updatePidConfig(driveConfig, driveMotor, drivePidConfig, "Drive");
            drivePidConfig.clearDirty();
        }
        if (config.isDirty()) {
            configureDriveMotor();
            configureSteerMotor();
            config.clearDirty();
        }
    }

    // ==================== CONTROL METHODS (Called by Commands to SET Targets) ====================

    @Override
    public void setTargets(LinearVelocity driveSpeed, Angle steerAngle) {
        controlState = ControlState.PID_CONTROL;
        driveTarget = driveSpeed.in(MetersPerSecond);
        steerTarget = steerAngle.in(Degrees);
    }

    @Override
    public void setMotors(double drivePercent, double steerPercent) {
        controlState = ControlState.OPEN_LOOP;
        driveTarget = drivePercent;
        steerTarget = steerPercent;
    }

    @Override
    public void stop() {
        controlState = ControlState.STOP;
    }

    // ==================== SENSOR METHODS (Queried to GET Current State) ====================

    @Override
    public LinearVelocity getDriveVelocity() {
        // Return cached velocity (already in m/s via conversion factor)
        return MetersPerSecond.of(driveVelocity);
    }

    @Override
    public Angle getSteerAngle() {
        // Return cached angle (already in degrees via conversion factor)
        return Degrees.of(steerAngle);
    }

    @Override
    public double getDrivePositionMeters() {
        // Read current position (already in meters via conversion factor)
        return driveEncoder.getPosition();
    }

    @Override
    public final LinearVelocity getMaxLinearVelocity() {
        // Calculate theoretical max speed based on NEO free speed and voltage compensation
        double maxMotorRpm = Constants.SparkConstants.FreeSpeedRPM * (Constants.SparkConstants.NominalVoltage / 12.0);
        return MetersPerSecond.of(motorRPMToMetersPerSecond(maxMotorRpm));
    }

    // ==================== DIAGNOSTIC & TESTING METHODS ====================

    // ==================== UTILITY METHODS ====================

    /**
     * Convert motor rotations to linear distance in meters.
     * Accounts for gear ratio and wheel circumference.
     * Used for encoder position conversion factor.
     */
    private double motorRotationsToMeters(double motorRotations) {
        double wheelRotations = motorRotations / config.driveRatio;
        double circumference = Math.PI * config.wheelDiameter.in(Meters);
        return wheelRotations * circumference;
    }

    /**
     * Update PID configuration at runtime (for dynamic tuning).
     * Called when PID gains are changed via NetworkTables or dashboard.
     */
    private void updatePidConfig(SparkMaxConfig sparkConfig, SparkMax motor, PidConfig pid, String motorType) {
        // Apply updated PID gains
        sparkConfig.closedLoop.p(pid.kP).i(pid.kI).d(pid.kD)
                .outputRange(pid.kMin, pid.kMax);
        sparkConfig.closedLoop.feedForward.kV(pid.kV);

        // Apply configuration without resetting safe parameters
        REVLibError error = motor.configure(sparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        if (error == REVLibError.kOk) {
            console(motorType + " motor PID updated: " + pid);
        } else {
            console("WARNING: " + motorType + " motor PID update failed: " + error);
        }
    }

    // ==================== SIMULATION ====================

    @Override
    public void updateSimState(double deltaTime, double batteryVoltage) {
        // SparkMax simulation support - REVLib handles this differently
        // For now, this is a no-op stub. Full implementation would require:
        // - DCMotorSim physics models for NEO motors
        // - REVLib simulation API integration
        // This is lower priority since TalonFX is the primary module type
    }
}
