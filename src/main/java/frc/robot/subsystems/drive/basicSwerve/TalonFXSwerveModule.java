package frc.robot.subsystems.drive.basicSwerve;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants;
import frc.robot.subsystems.drive.basicSwerve.config.ModuleConfig;
import frc.robot.subsystems.drive.basicSwerve.config.PidConfig;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

/**
 * TalonFX-based swerve module implementation.
 * Uses CTRE TalonFX (Kraken X60) motor controllers and CANcoder on CANivore bus.
 *
 * ORGANIZATION:
 * - Fields (hardware and state)
 * - Constructor (runs once at startup)
 * - Configuration methods (called from constructor)
 * - Periodic (runs every 20ms - the main loop)
 * - Control methods (called by commands to SET targets)
 * - Sensor methods (queried to GET current state)
 * - Diagnostic methods (for testing)
 */
public class TalonFXSwerveModule extends SwerveModuleBase {

    // ==================== HARDWARE ====================

    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder steerCANCoder;

    // Control requests (reusable objects for performance)
    private final VelocityVoltage driveVelocityControl;
    private final PositionVoltage steerPositionControl;

    // ==================== STATE ====================

    // Cached sensor values (updated in periodic)
    private double driveVelocityRPS;
    private double steerAngleDegrees;

    // Target values (set by control methods, sent in periodic based on state)
    private double driveTarget = 0;  // RPS for PID_CONTROL, percent for DIRECT_CONTROL
    private double steerTarget = 0;  // Rotations for PID_CONTROL, percent for DIRECT_CONTROL

    // ==================== CONSTRUCTOR (Runs Once at Startup) ====================

    public TalonFXSwerveModule(int index, ModuleConfig moduleConfig, PidConfig drivePidConfig, PidConfig steerPidConfig) {
        super(index, moduleConfig, drivePidConfig, steerPidConfig);

        // CANBus setup
        String canBusName = Constants.Drive.canBusName;
        String busName = (canBusName == null || canBusName.isEmpty()) ? "" : canBusName;
        CANBus canBus = new CANBus(busName);
        console("Configured for CANivore: \"" + busName + "\"");

        // Create hardware objects
        driveMotor = new TalonFX(config.driveID, canBus);
        steerMotor = new TalonFX(config.steerID, canBus);
        steerCANCoder = new CANcoder(config.encoderID, canBus);

        // Configure hardware
        configureCancoderMagnetOffset(steerCANCoder);  // Zero the CANcoder offset
        configureDriveMotor();                          // Set up drive PID, limits, etc.
        configureSteerMotor();                          // Set up steer PID, limits, etc.
        syncSteerEncoderWithCancoder();                 // Sync steer motor encoder to absolute position

        // Create control requests (reusable for performance)
        driveVelocityControl = new VelocityVoltage(0).withSlot(0);
        steerPositionControl = new PositionVoltage(0).withSlot(0);

        console("TalonFX module initialized");
    }

    // ==================== CONFIGURATION (Called from Constructor) ====================

    private void configureDriveMotor() {
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();

        // PID configuration (Slot 0)
        // Using VelocityVoltage control - units are wheel rotations per second
        // kP: Volts per (wheel rotation/sec) of velocity error
        // kI: Volts per (wheel rotation/sec) of integrated error
        // kD: Volts per (wheel rotation/sec^2) of error derivative
        // kV: Volts per (wheel rotation/sec) - feedforward for target velocity
        Slot0Configs slot0 = driveConfig.Slot0;
        slot0.kP = drivePidConfig.kP;
        slot0.kI = drivePidConfig.kI;
        slot0.kD = drivePidConfig.kD;

        // Auto-calculate feedforward gain (kV is in volts per RPS at wheel)
        double krakenFreeSpeedRPM = Constants.TalonConstants.krakenFreeSpeedRPM;
        double maxWheelRPM = krakenFreeSpeedRPM / config.driveRatio;
        double maxWheelRPS = maxWheelRPM / 60.0;
        drivePidConfig.kV = 12.0 / maxWheelRPS;

        slot0.kV = drivePidConfig.kV;
        slot0.kS = drivePidConfig.kS;
        slot0.kA = drivePidConfig.kA;

        // Current limits
        CurrentLimitsConfigs currentLimits = driveConfig.CurrentLimits;
        currentLimits.StatorCurrentLimit = Constants.TalonConstants.driveStatorCurrentLimit;
        currentLimits.StatorCurrentLimitEnable = true;

        // Motor output configuration
        MotorOutputConfigs motorOutput = driveConfig.MotorOutput;
        motorOutput.Inverted = config.driveInverted ?
            InvertedValue.Clockwise_Positive: InvertedValue.CounterClockwise_Positive ;
        motorOutput.NeutralMode = config.driveCoast ?
            NeutralModeValue.Coast : NeutralModeValue.Brake;

        // Feedback configuration (tells TalonFX about gear ratio)
        // SensorToMechanismRatio = sensor rotations per mechanism rotation
        // For drive: config.driveRatio motor rotations = 1 wheel rotation
        // Example: 6.75 motor rotations per 1 wheel rotation
        FeedbackConfigs feedback = driveConfig.Feedback;
        feedback.SensorToMechanismRatio = config.driveRatio;

        // Apply configuration and check status
        var driveStatus = driveMotor.getConfigurator().apply(driveConfig);
        console("Drive motor config apply status: " + driveStatus);
        if (!driveStatus.isOK()) {
            console("WARNING: Drive motor configuration failed: " + driveStatus);
        }
    }

    private void configureSteerMotor() {
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();

        // PID configuration (Slot 0)
        // Using PositionVoltage control - units are wheel rotations
        // kP: Volts per wheel rotation of position error
        // kI: Volts per wheel rotation of integrated error
        // kD: Volts per (wheel rotation/sec) of error derivative
        Slot0Configs slot0 = steerConfig.Slot0;
        slot0.kP = steerPidConfig.kP;
        slot0.kI = steerPidConfig.kI;
        slot0.kD = steerPidConfig.kD;
        slot0.kV = 0;  // No feedforward for steer (position control doesn't need it)
        slot0.kS = steerPidConfig.kS;

        // Current limits
        CurrentLimitsConfigs currentLimits = steerConfig.CurrentLimits;
        currentLimits.StatorCurrentLimit = Constants.TalonConstants.steerStatorCurrentLimit;
        currentLimits.StatorCurrentLimitEnable = true;

        // Motor output configuration
        MotorOutputConfigs motorOutput = steerConfig.MotorOutput;
        motorOutput.Inverted = config.steerInverted ?
            InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        motorOutput.NeutralMode = config.steerCoast ?
            NeutralModeValue.Coast : NeutralModeValue.Brake;

        // Feedback configuration (tells TalonFX about gear ratio)
        // We use motor's internal encoder (not remote CANcoder)
        // SensorToMechanismRatio = sensor rotations per mechanism rotation
        // For steer: config.steerRatio motor rotations = 1 wheel rotation
        // Example: 21.429 motor rotations per 1 wheel rotation
        FeedbackConfigs feedback = steerConfig.Feedback;
        feedback.SensorToMechanismRatio = config.steerRatio;

        // Closed loop configuration for continuous rotation
        // Allows the motor to wrap around (0° = 360°)
        ClosedLoopGeneralConfigs closedLoop = steerConfig.ClosedLoopGeneral;
        closedLoop.ContinuousWrap = true;

        // Apply configuration and check status
        var steerStatus = steerMotor.getConfigurator().apply(steerConfig);
        console("Steer motor config apply status: " + steerStatus);
        if (!steerStatus.isOK()) {
            console("WARNING: Steer motor configuration failed: " + steerStatus);
        }

        // Log configured PID gains for diagnostic purposes
        console("Steer PID Slot0 - kP: " + steerPidConfig.kP +
                ", kI: " + steerPidConfig.kI +
                ", kD: " + steerPidConfig.kD +
                ", kS: " + steerPidConfig.kS);
        console("Steer SensorToMechanismRatio: " + config.steerRatio +
                " (motor rotations per wheel rotation)");
    }

    private void syncSteerEncoderWithCancoder() {
        // Read absolute position from CANcoder
        double cancoderRotations = steerCANCoder.getAbsolutePosition().getValueAsDouble();
        double cancoderDegrees = cancoderRotations * 360.0;

        // Calculate module-relative wheel position (subtract offset)
        double wheelPositionDegrees = cancoderDegrees - steerOffset.in(Degrees);

        // CRITICAL: Normalize to [-180, 180] range before converting to rotations
        wheelPositionDegrees = normalizeDegrees(wheelPositionDegrees);

        double wheelPositionRotations = wheelPositionDegrees / 360.0;

        // Set the motor encoder position to match
        // After this, motor position = 0 means module pointing at 0° (forward)
        steerMotor.setPosition(wheelPositionRotations);

        // Log sync information
        console("Synced steer encoder: CANcoder=" + cancoderDegrees +
                "° → wheel=" + wheelPositionDegrees +
                "° (" + wheelPositionRotations + " rot)");
        console("Offset semantics: module angle 0° = CANcoder " + steerOffset.in(Degrees) + "°");
        console("Sanity check: After boot, commanding 0° should point the wheel to the defined zero direction");
    }

    // ==================== PERIODIC (Runs Every 20ms - This is the Main Loop!) ====================

    @Override
    public void periodic() {
        // 1. READ SENSORS
        // Update cached sensor values (in mechanism units: wheel RPS and wheel rotations)
        driveVelocityRPS = driveMotor.getVelocity().getValueAsDouble();  // Wheel RPS

        double wheelRotations = steerMotor.getPosition().getValueAsDouble();  // Wheel rotations
        steerAngleDegrees = normalizeDegrees(wheelRotations * 360.0);  // Convert to degrees and normalize

        // 2. SEND CONTROL SIGNALS BASED ON STATE
        // Control methods set the state and target; here we execute based on state

        switch (controlState) {
            case STOP:
                driveMotor.stopMotor();
                steerMotor.stopMotor();
                break;
            case PID_CONTROL:
                var driveStatus = driveMotor.setControl(driveVelocityControl.withVelocity(driveTarget));
                var steerStatus = steerMotor.setControl(steerPositionControl.withPosition(steerTarget));
                if (!driveStatus.isOK()) {
                    console("Drive setControl failed: " + driveStatus);
                }
                if (!steerStatus.isOK()) {
                    console("Steer setControl failed: " + steerStatus);
                }
                break;
            case OPEN_LOOP:
                driveMotor.set(driveTarget);
                steerMotor.set(steerTarget);
                break;
        }

        // 3. LOG TELEMETRY
        telemetry();

        if (Constants.Toggles.usePidTuner) {
            if (drivePidConfig.isDirty()) {
                configureDriveMotor();
                drivePidConfig.clearDirty();
            }
            if (steerPidConfig.isDirty()) {
                configureSteerMotor();
                steerPidConfig.clearDirty();
            }
        }
    }

    // ==================== CONTROL METHODS (Called by Commands to SET Targets) ====================

    @Override
    public void setTargets(LinearVelocity driveSpeed, Angle steerAngle) {
        controlState = ControlState.PID_CONTROL;

        // Set drive target
        driveTarget = metersPerSecondToWheelRPS(driveSpeed.in(MetersPerSecond));

        // Set steer target using nearest equivalent rotation
        // Convert desired angle to rotations (normalized 0..1)
        double desired = steerAngle.in(Rotations);

        // Get current position (unbounded - can be > 1 or < 0 with continuous wrap)
        double current = steerMotor.getPosition().getValueAsDouble();

        // Calculate shortest path to target using IEEEremainder (returns value in [-0.5, 0.5])
        // This ensures we always take the shortest rotation to the target
        double delta = Math.IEEEremainder(desired - current, 1.0);

        // Command the nearest equivalent rotation to current position
        // This keeps the error small and unambiguous with continuous wrap
        steerTarget = current + delta;
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
        // Convert cached wheel RPS to m/s
        return MetersPerSecond.of(wheelRPSToMetersPerSecond(driveVelocityRPS));
    }

    @Override
    public Angle getSteerAngle() {
        // Return cached steer angle (already in degrees)
        return Degrees.of(steerAngleDegrees);
    }

    @Override
    public double getDrivePositionMeters() {
        // Read current wheel position and convert to meters
        double wheelRotations = driveMotor.getPosition().getValueAsDouble();
        double wheelCircumference = Math.PI * config.wheelDiameter.in(Meters);
        return wheelRotations * wheelCircumference;
    }

    @Override
    public LinearVelocity getMaxLinearVelocity() {
        // Calculate theoretical max speed based on Kraken free speed
        double krakenFreeSpeedRPM = Constants.TalonConstants.krakenFreeSpeedRPM;
        double maxWheelRPM = krakenFreeSpeedRPM / config.driveRatio;
        double maxWheelRPS = maxWheelRPM / 60.0;
        return MetersPerSecond.of(wheelRPSToMetersPerSecond(maxWheelRPS));
    }

    // ==================== DIAGNOSTIC & TESTING METHODS ====================

    @Override
    public void telemetry() {
        // Call base class telemetry (logs common values)
        super.telemetry();

        // Log TalonFX-specific telemetry
        String basePath = "Drive/Modules/" + name;

        // Local cached values
        Logger.recordOutput(basePath + "/Drive/localSteerAngle", steerAngleDegrees);
        Logger.recordOutput(basePath + "/Drive/localDriveVelocity", driveVelocityRPS);
        Logger.recordOutput(basePath + "/Drive/localDriveTarget", driveTarget);
        Logger.recordOutput(basePath + "/Steer/localSteerTarget", steerTarget);

        // Drive motor closed-loop telemetry
        Logger.recordOutput(basePath + "/Drive/ClosedLoopReference", driveMotor.getClosedLoopReference().getValueAsDouble());
        Logger.recordOutput(basePath + "/Drive/ClosedLoopError", driveMotor.getClosedLoopError().getValueAsDouble());
        Logger.recordOutput(basePath + "/Drive/ClosedLoopOutput", driveMotor.getClosedLoopOutput().getValueAsDouble());

        // Steer motor closed-loop telemetry (for debugging PID)
        Logger.recordOutput(basePath + "/Steer/ClosedLoopReference", steerMotor.getClosedLoopReference().getValueAsDouble());
        Logger.recordOutput(basePath + "/Steer/ClosedLoopError", steerMotor.getClosedLoopError().getValueAsDouble());
        Logger.recordOutput(basePath + "/Steer/ClosedLoopOutput", steerMotor.getClosedLoopOutput().getValueAsDouble());
        Logger.recordOutput(basePath + "/Steer/RawPosition", steerMotor.getPosition().getValueAsDouble());
    }
}
