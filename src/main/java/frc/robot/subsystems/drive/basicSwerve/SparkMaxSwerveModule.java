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
import frc.robot.subsystems.drive.basicSwerve.config.PidConfig;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

/**
 * SparkMax-based swerve module implementation.
 * Uses REV Robotics SparkMax motor controllers and CTRE CANcoder for absolute position.
 */
public class SparkMaxSwerveModule extends SwerveModuleBase {
    private final CANcoder steerCANCoder;
    private final SparkMax driveMotor;
    private final SparkMax steerMotor;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steerEncoder;
    private final SparkMaxConfig driveConfig;
    private final SparkMaxConfig steerConfig;
    private final SparkClosedLoopController steerController;
    private final SparkClosedLoopController driveController;

    // Cached telemetry values (REV encoders are slow to query)
    private double driveVelocity;
    private double steerAngle;

    public SparkMaxSwerveModule(int index, ModuleConfig moduleConfig, PidConfig drivePidConfig, PidConfig steerPidConfig) {
        super(index, moduleConfig, drivePidConfig, steerPidConfig);

        // CANcoder setup
        steerCANCoder = new CANcoder(config.encoderID);
        configureCancoderMagnetOffset(steerCANCoder);

        // Motor creation
        driveMotor = new SparkMax(config.driveID, SparkBase.MotorType.kBrushless);
        steerMotor = new SparkMax(config.steerID, SparkBase.MotorType.kBrushless);

        // Initialize config objects
        driveConfig = new SparkMaxConfig();
        steerConfig = new SparkMaxConfig();

        // Delegate configuration
        configureDriveMotor();
        configureSteerMotor();

        // Cache references
        driveController = driveMotor.getClosedLoopController();
        driveEncoder = driveMotor.getEncoder();
        steerController = steerMotor.getClosedLoopController();
        steerEncoder = steerMotor.getEncoder();

        // Sync steer encoder with CANcoder
        syncSteerEncoderWithCancoder();

        console("SparkMax module initialized");
    }

    private void configureDriveMotor() {
        SparkMaxConfig globalConfig = new SparkMaxConfig();
        globalConfig.smartCurrentLimit(Constants.SparkConstants.CurrentLimit)
                    .voltageCompensation(Constants.SparkConstants.NominalVoltage);

        EncoderConfig driveEncoderConfig = new EncoderConfig();
        driveEncoderConfig.velocityConversionFactor(motorRPMToMetersPerSecond(1.0));

        driveConfig.apply(globalConfig)
                   .openLoopRampRate(Constants.Drive.driveOpenLoopRampRate)
                   .inverted(config.driveInverted);
        driveConfig.apply(driveEncoderConfig);

        // Calculate feedforward gain
        drivePidConfig.kV = 1.0 / getMaxLinearVelocity().in(MetersPerSecond);

        driveConfig.closedLoop
                .p(drivePidConfig.kP).i(drivePidConfig.kI).d(drivePidConfig.kD)
                .outputRange(drivePidConfig.kMin, drivePidConfig.kMax);
        driveConfig.closedLoop.feedForward.kV(drivePidConfig.kV);

        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureSteerMotor() {
        SparkMaxConfig globalConfig = new SparkMaxConfig();
        globalConfig.smartCurrentLimit(Constants.SparkConstants.CurrentLimit)
                    .voltageCompensation(Constants.SparkConstants.NominalVoltage);

        EncoderConfig steerEncoderConfig = new EncoderConfig();
        steerEncoderConfig.positionConversionFactor(steerRotationsToDegrees(1.0));

        steerConfig.apply(globalConfig)
                   .openLoopRampRate(Constants.Drive.steerOpenLoopRampRate)
                   .inverted(config.steerInverted);
        steerConfig.apply(steerEncoderConfig);

        steerPidConfig.kV = 0; // No feedforward for steer
        steerConfig.closedLoop
                .p(steerPidConfig.kP).i(steerPidConfig.kI).d(steerPidConfig.kD)
                .outputRange(steerPidConfig.kMin, steerPidConfig.kMax)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(-180, 180);
        steerConfig.closedLoop.feedForward.kV(steerPidConfig.kV);

        steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void syncSteerEncoderWithCancoder() {
        double cancoderPosition = steerCANCoder.getAbsolutePosition().getValue().in(Degrees);
        steerEncoder.setPosition(cancoderPosition - steerOffset.in(Degrees));
    }

    @Override
    public void periodic() {
        // Update cached sensor values (REV encoders are slow to query repeatedly)
        driveVelocity = driveEncoder.getVelocity();
        steerAngle = normalizeDegrees(steerEncoder.getPosition());

        telemetry();

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

    @Override
    public void driveMotorSet(double voltage) {
        driveMotor.set(voltage);
    }

    @Override
    public void steerMotorSet(double voltage) {
        steerMotor.set(voltage);
    }

    @Override
    public LinearVelocity getDriveVelocity() {
        return MetersPerSecond.of(driveVelocity);
    }

    @Override
    public void setDriveVelocity(LinearVelocity speed) {
        driveController.setSetpoint(speed.in(MetersPerSecond), SparkBase.ControlType.kVelocity);
    }

    @Override
    public Angle getSteerAngle() {
        return Degrees.of(steerAngle);
    }

    @Override
    public void setSteerAngle(Angle angle) {
        steerController.setSetpoint(angle.in(Degrees), SparkBase.ControlType.kPosition);
    }

    @Override
    public final LinearVelocity getMaxLinearVelocity() {
        // Calculate actual max RPM based on voltage compensation
        double maxMotorRpm = Constants.SparkConstants.FreeSpeedRPM * (Constants.SparkConstants.NominalVoltage / 12.0);
        return MetersPerSecond.of(motorRPMToMetersPerSecond(maxMotorRpm));
    }

    @Override
    public void stop() {
        driveMotorSet(0);
    }

    private void updatePidConfig(SparkMaxConfig sparkConfig, SparkMax motor, PidConfig pid, String motorType) {
        sparkConfig.closedLoop.p(pid.kP).i(pid.kI).d(pid.kD)
                .outputRange(pid.kMin, pid.kMax);
        sparkConfig.closedLoop.feedForward.kV(pid.kV);

        REVLibError error = motor.configure(sparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        if (error == REVLibError.kOk) {
            console(motorType + " motor configured successfully: " + pid);
        } else {
            console("Error configuring " + motorType + " motor: " + error);
        }
    }
}
