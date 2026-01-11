package frc.robot.subsystems.drive.config;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

/**
 * SparkMax-based swerve module implementation.
 * Uses REV Robotics SparkMax motor controllers and CTRE CANcoder for absolute position.
 */
public class SparkMaxSwerveModule extends SwerveModuleBase {

    public final CANcoder steerCANCoder;
    public final SparkMax driveMotor;
    public final SparkMax steerMotor;
    public final RelativeEncoder driveEncoder;
    public final RelativeEncoder steerEncoder;
    private final SparkMaxConfig driveConfig;
    private final SparkMaxConfig steerConfig;
    private final SparkClosedLoopController steerController;
    private final SparkClosedLoopController driveController;
    private double driveVelocity;
    private double tgtDriveVelocity = 0;
    private double steerAngle;
    private double tgtSteerAngle = 0;

    public SparkMaxSwerveModule(int index, ModuleConfig moduleConfig, PidConfig drivePidConfig, PidConfig steerPidConfig) {
        super(index, moduleConfig, drivePidConfig, steerPidConfig);

        // CANCODER - read only!
        steerCANCoder = new CANcoder(config.encoderID);

        // Zero the magnetic offset to ensure we obliterate any prior stored values and read raw values
        MagnetSensorConfigs magnetConfigs = new MagnetSensorConfigs();
        magnetConfigs.MagnetOffset = 0.0;
        steerCANCoder.getConfigurator().apply(magnetConfigs);

        Angle canOffset = Degrees.of(steerOffset);
        // /CANCODER

        // MOTORS
        SparkMaxConfig globalConfig = new SparkMaxConfig();

        globalConfig.smartCurrentLimit(Constants.SparkConstants.CurrentLimit).voltageCompensation(Constants.SparkConstants.NominalVoltage);

        // MOTORS: DRIVE MOTOR
        driveMotor = new SparkMax(config.driveID, SparkBase.MotorType.kBrushless);
        driveConfig = new SparkMaxConfig();
        EncoderConfig driveEncoderConfig = new EncoderConfig();

        driveConfig.apply(globalConfig).openLoopRampRate(Constants.Drive.driveOpenLoopRampRate).inverted(config.driveInverted);

        driveEncoderConfig.velocityConversionFactor(driveRPMToMetersPerSecond(1.0));

        driveConfig.apply(driveEncoderConfig);

        drivePidConfig.kV = 1.0 / getMaxLinearVelocity().in(MetersPerSecond);

        driveConfig.closedLoop
                .p(drivePidConfig.kP).i(drivePidConfig.kI).d(drivePidConfig.kD)
                .velocityFF(drivePidConfig.kV).outputRange(drivePidConfig.kMin, drivePidConfig.kMax);

        driveMotor.configure(driveConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        driveController = driveMotor.getClosedLoopController();
        driveEncoder = driveMotor.getEncoder();
        // /MOTORS: DRIVE MOTOR

        // MOTORS: STEER MOTOR
        steerMotor = new SparkMax(config.steerID, SparkBase.MotorType.kBrushless);
        steerConfig = new SparkMaxConfig();
        EncoderConfig steerEncoderConfig = new EncoderConfig();

        steerConfig.apply(globalConfig).openLoopRampRate(Constants.Drive.steerOpenLoopRampRate).inverted(config.steerInverted);

        steerEncoderConfig.positionConversionFactor(steerRotationsToDegrees(1.0));
        steerConfig.apply(steerEncoderConfig);

        steerPidConfig.kV = 0; // probably want nothing here for the steer
        steerConfig.closedLoop
                .p(steerPidConfig.kP).i(steerPidConfig.kI).d(steerPidConfig.kD)
                .velocityFF(steerPidConfig.kV).outputRange(steerPidConfig.kMin, steerPidConfig.kMax)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(-180, 180);

        steerMotor.configure(steerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        steerController = steerMotor.getClosedLoopController();
        steerEncoder = steerMotor.getEncoder();

        steerEncoder.setPosition(steerCANCoder.getAbsolutePosition().getValue().in(Degrees) - canOffset.in(Degrees));

        // /MOTORS: STEER MOTOR
        // /MOTORS
    }

    @Override
    public void periodic() {
        telemetry();

        if (steerPidConfig.isDirty()) {
            updatePidConfig(steerConfig, steerMotor, steerPidConfig, "Steer");
        }
        if (drivePidConfig.isDirty()) {
            updatePidConfig(driveConfig, driveMotor, drivePidConfig, "Drive");
        }
    }

    @Override
    protected void telemetry() {
        // Update internal cached values for getDriveVelocity() and getSteerAngle()
        driveVelocity = driveEncoder.getVelocity();
        double rawSteerAngle = steerEncoder.getPosition();
        steerAngle = normalizeDegrees(rawSteerAngle);

        // Log using base class implementation (calls getDriveVelocity() and getSteerAngle())
        super.telemetry();
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
    public double getDriveVelocity() {
        return driveVelocity;
    }

    @Override
    public void setDriveVelocity(LinearVelocity speed) {
        tgtDriveVelocity = speed.in(MetersPerSecond);
        driveController.setReference(tgtDriveVelocity, SparkBase.ControlType.kVelocity);
    }

    @Override
    public double getSteerAngle() {
        return steerAngle;
    }

    @Override
    public void setSteerAngle(Angle angle) {
        tgtSteerAngle = angle.in(Degrees);
        steerController.setReference(tgtSteerAngle, SparkBase.ControlType.kPosition);
    }

    @Override
    public final LinearVelocity getMaxLinearVelocity() {
        // Calculate actual max RPM based on voltage compensation
        double maxRpm = Constants.SparkConstants.FreeSpeedRPM * (Constants.SparkConstants.NominalVoltage / 12.0);

        return MetersPerSecond.of(driveRPMToMetersPerSecond(maxRpm));
    }

    @Override
    public void setSwerveModuleState(SwerveModuleState state) {
        if (Constants.Drive.angleOptimize) {
            state.optimize(new Rotation2d(Degrees.of(steerAngle)));
        }

        if (Constants.Drive.cosineOptimize) {
            state.cosineScale(new Rotation2d(Degrees.of(steerAngle)));
        }

        setDriveVelocity(MetersPerSecond.of(state.speedMetersPerSecond));
        setSteerAngle(Degrees.of(state.angle.getDegrees()));
    }

    @Override
    public void stop() {
        driveMotorSet(0);
    }

    private void updatePidConfig(SparkMaxConfig config, SparkMax motor, PidConfig pid, String motorType) {
        config.closedLoop.p(pid.kP).i(pid.kI).d(pid.kD).velocityFF(pid.kV) // Using kV as feedforward term
                .outputRange(pid.kMin, pid.kMax);

        REVLibError error = motor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        if (error == REVLibError.kOk) {
            console(motorType + " motor configured successfully: " + pid);
        } else {
            console("Error configuring " + motorType + " motor: " + error.toString());
        }
    }

    private double driveRPMToMetersPerSecond(double rpm) {
        double INCHES_TO_METERS = 0.0254;  // Standard conversion
        return rpm // rotations per minute
                * config.wheelDiameter // inches per rotation
                * INCHES_TO_METERS // convert to meters
                * Math.PI // circumference/diameter
                * (1.0 / 60.0) // convert from per minute to per second
                * (1.0 / config.driveRatio); // wheel rotations per motor rotation
    }

    private double steerRotationsToDegrees(double rotations) {
        return rotations // input rotations
                * 360.0 // degrees per rotation
                * (1.0 / config.steerRatio); // output rotations per input rotation
    }
}
