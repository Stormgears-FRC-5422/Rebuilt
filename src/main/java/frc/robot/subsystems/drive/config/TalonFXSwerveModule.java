package frc.robot.subsystems.drive.config;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.*;

/**
 * TalonFX-based swerve module implementation.
 * Uses CTRE TalonFX (Kraken) motor controllers and CANcoder on CANivore bus.
 */
public class TalonFXSwerveModule extends SwerveModuleBase {

    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder steerCANCoder;

    private final VelocityVoltage driveVelocityControl;
    private final PositionVoltage steerPositionControl;

    private final String canBusName;

    public TalonFXSwerveModule(int index, ModuleConfig moduleConfig, PidConfig drivePidConfig, PidConfig steerPidConfig) {
        super(index, moduleConfig, drivePidConfig, steerPidConfig);

        canBusName = Constants.Drive.canBusName;
        String busName = (canBusName == null || canBusName.isEmpty()) ? "" : canBusName;
        CANBus canBus = new CANBus(busName);

        console("Configured for CANivore: \"" + busName + "\"");

        driveMotor = new TalonFX(config.driveID, canBus);
        steerMotor = new TalonFX(config.steerID, canBus);
        steerCANCoder = new CANcoder(config.encoderID, canBus);

        configureCancoder();
        configureDriveMotor();
        configureSteerMotor();

        // Create control requests (reusable for performance)
        driveVelocityControl = new VelocityVoltage(0).withSlot(0);
        steerPositionControl = new PositionVoltage(0).withSlot(0);

        console("TalonFX module initialized on " + canBusName);
    }

    private void configureCancoder() {
        MagnetSensorConfigs magnetConfigs = new MagnetSensorConfigs();
        // Convert degrees to rotations for TalonFX
        magnetConfigs.MagnetOffset = config.encoderOffset / 360.0;
        steerCANCoder.getConfigurator().apply(magnetConfigs);
    }

    private void configureDriveMotor() {
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();

        // PID configuration
        Slot0Configs slot0 = driveConfig.Slot0;
        slot0.kP = drivePidConfig.kP;
        slot0.kI = drivePidConfig.kI;
        slot0.kD = drivePidConfig.kD;
        slot0.kV = drivePidConfig.kV;
        slot0.kS = drivePidConfig.kS;

        // Current limits
        CurrentLimitsConfigs currentLimits = driveConfig.CurrentLimits;
        currentLimits.StatorCurrentLimit = Constants.TalonConstants.driveStatorCurrentLimit;
        currentLimits.StatorCurrentLimitEnable = true;

        // Motor output
        MotorOutputConfigs motorOutput = driveConfig.MotorOutput;
        motorOutput.Inverted = config.driveInverted ?
            InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        motorOutput.NeutralMode = config.driveCoast ?
            NeutralModeValue.Coast : NeutralModeValue.Brake;

        // Gear ratio (sensor-to-mechanism for drive wheels)
        FeedbackConfigs feedback = driveConfig.Feedback;
        feedback.SensorToMechanismRatio = config.driveRatio;

        driveMotor.getConfigurator().apply(driveConfig);
    }

    private void configureSteerMotor() {
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();

        // PID configuration
        Slot0Configs slot0 = steerConfig.Slot0;
        slot0.kP = steerPidConfig.kP;
        slot0.kI = steerPidConfig.kI;
        slot0.kD = steerPidConfig.kD;
        slot0.kV = steerPidConfig.kV;
        slot0.kS = steerPidConfig.kS;

        // Current limits
        CurrentLimitsConfigs currentLimits = steerConfig.CurrentLimits;
        currentLimits.StatorCurrentLimit = Constants.TalonConstants.steerStatorCurrentLimit;
        currentLimits.StatorCurrentLimitEnable = true;

        // Motor output
        MotorOutputConfigs motorOutput = steerConfig.MotorOutput;
        motorOutput.Inverted = config.steerInverted ?
            InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        motorOutput.NeutralMode = config.steerCoast ?
            NeutralModeValue.Coast : NeutralModeValue.Brake;

        // Remote CANcoder feedback
        FeedbackConfigs feedback = steerConfig.Feedback;
        feedback.FeedbackRemoteSensorID = steerCANCoder.getDeviceID();
        feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        feedback.SensorToMechanismRatio = config.steerRatio;
        feedback.RotorToSensorRatio = 1.0;

        // Closed loop configuration for continuous rotation
        ClosedLoopGeneralConfigs closedLoop = steerConfig.ClosedLoopGeneral;
        closedLoop.ContinuousWrap = true; // Enable position wrapping at ±0.5 rotations (±180°)

        steerMotor.getConfigurator().apply(steerConfig);
    }

    @Override
    public void setDriveVelocity(LinearVelocity speed) {
        // Convert m/s to rotations/second of the wheel
        double wheelCircumference = Math.PI * config.wheelDiameter * 0.0254; // inches to meters
        double targetRPS = speed.in(MetersPerSecond) / wheelCircumference;

        driveMotor.setControl(driveVelocityControl.withVelocity(targetRPS));
    }

    @Override
    public void setSteerAngle(Angle angle) {
        // Convert degrees to rotations
        double targetRotations = angle.in(Degrees) / 360.0;

        steerMotor.setControl(steerPositionControl.withPosition(targetRotations));
    }

    @Override
    public void setSwerveModuleState(SwerveModuleState state) {
        if (Constants.Drive.angleOptimize) {
            state.optimize(new Rotation2d(Degrees.of(getSteerAngle())));
        }

        if (Constants.Drive.cosineOptimize) {
            state.cosineScale(new Rotation2d(Degrees.of(getSteerAngle())));
        }

        setDriveVelocity(MetersPerSecond.of(state.speedMetersPerSecond));
        setSteerAngle(Degrees.of(state.angle.getDegrees()));
    }

    @Override
    public double getDriveVelocity() {
        double rps = driveMotor.getVelocity().getValueAsDouble(); // rotations per second
        double wheelCircumference = Math.PI * config.wheelDiameter * 0.0254;
        return rps * wheelCircumference; // m/s
    }

    @Override
    public double getSteerAngle() {
        double rotations = steerMotor.getPosition().getValueAsDouble();
        return normalizeDegrees(rotations * 360.0); // rotations to degrees, normalized
    }

    @Override
    public LinearVelocity getMaxLinearVelocity() {
        // Kraken X60 free speed: ~6000 RPM
        double krakenFreeSpeedRPM = 6000.0;
        double maxRPM = krakenFreeSpeedRPM / config.driveRatio;
        double maxRPS = maxRPM / 60.0;
        double wheelCircumference = Math.PI * config.wheelDiameter * 0.0254;
        return MetersPerSecond.of(maxRPS * wheelCircumference);
    }

    @Override
    public void periodic() {
        telemetry();

        // Check for PID updates (dirty flag pattern)
        if (drivePidConfig.isDirty()) {
            configureDriveMotor();
            drivePidConfig.clearDirty();
        }
        if (steerPidConfig.isDirty()) {
            configureSteerMotor();
            steerPidConfig.clearDirty();
        }
        if (config.isDirty()) {
            configureDriveMotor();
            configureSteerMotor();
            config.clearDirty();
        }
    }

    @Override
    public void stop() {
        driveMotor.stopMotor();
        steerMotor.stopMotor();
    }

    // Diagnostic methods (for DiagnosticSwerve compatibility)
    @Override
    public void driveMotorSet(double voltage) {
        driveMotor.set(voltage);
    }

    @Override
    public void steerMotorSet(double voltage) {
        steerMotor.set(voltage);
    }
}
