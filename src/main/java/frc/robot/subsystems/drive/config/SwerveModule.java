package frc.robot.subsystems.drive.config;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
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
import frc.robot.Options.DriveOptions;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class SwerveModule {

    public final int index;
    public final CANcoder steerCANCoder;
    public final SparkMax driveMotor;
    public final SparkMax steerMotor;
    public final RelativeEncoder driveEncoder;
    public final RelativeEncoder steerEncoder;
    public final ModuleConfig config;
    public final String name;
    private final SparkMaxConfig driveConfig;
    private final SparkMaxConfig steerConfig;
    private final SparkClosedLoopController steerController;
    private final SparkClosedLoopController driveController;
    private final PidConfig dPid;
    private final PidConfig sPid;
    public double steerOffset;
    private double driveVelocity;
    private double tgtDriveVelocity = 0;
    private double steerAngle;
    private double tgtSteerAngle = 0;
    private final DriveOptions options;

    public SwerveModule(int index, ModuleConfig moduleConfig, PidConfig dPid, PidConfig sPid) {
        options = DriveOptions.create();

        this.index = index;
        config = moduleConfig;
        this.dPid = dPid;
        this.sPid = sPid;
        name = config.name;
        steerOffset = config.encoderOffset;

		// CANCoder
        steerCANCoder = new CANcoder(config.encoderID);

        // Zero the magnetic offset to ensure we obliterate any prior stored values and read raw values
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.MagnetOffset = 0.0;
        StatusCode status = steerCANCoder.getConfigurator().apply(canCoderConfig, 0.050);
        if (status.isOK()) {
            console("Successfully configured CANcoder " + config.encoderID + " magnetic offset");
        } else {
            console("Failed to configure CANcoder " + config.encoderID + " magnetic offset: " + status.toString());
        }

        Angle canOffset = Degrees.of(steerOffset);       

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

        dPid.kV = 1.0 / getMaxLinearVelocity().in(MetersPerSecond);

        driveConfig.closedLoop
                .p(dPid.kP).i(dPid.kI).d(dPid.kD)
                .velocityFF(dPid.kV).outputRange(dPid.kMin, dPid.kMax);

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

        sPid.kV = 0; // probably want nothing here for the steer
        steerConfig.closedLoop
                .p(sPid.kP).i(sPid.kI).d(sPid.kD)
                .velocityFF(sPid.kV).outputRange(sPid.kMin, sPid.kMax)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(-180, 180);

        steerMotor.configure(steerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        steerController = steerMotor.getClosedLoopController();
        steerEncoder = steerMotor.getEncoder();

        steerEncoder.setPosition(steerCANCoder.getAbsolutePosition().getValue().in(Degrees) - canOffset.in(Degrees));

        // /MOTORS: STEER MOTOR
        // /MOTORS
    }

    public void periodic() {
        telemetry();

        if (sPid.isDirty()) {
            updatePidConfig(steerConfig, steerMotor, sPid, "Steer");
        }
        if (dPid.isDirty()) {
            updatePidConfig(driveConfig, driveMotor, dPid, "Drive");
        }
    }

    public void telemetry() {
        driveVelocity = driveEncoder.getVelocity();
        double rawSteerAngle = steerEncoder.getPosition();
        steerAngle = normalizeDegrees(rawSteerAngle);

        Angle canCoderAngle = steerCANCoder.getPosition().getValue();

        Logger.recordOutput("Mod " + name + "/driveOutput", driveMotor.get());
        Logger.recordOutput("Mod " + name + "/driveVelocity", driveVelocity);
        Logger.recordOutput("Mod " + name + "/targetDriveVelocity", tgtDriveVelocity);
        Logger.recordOutput("Mod " + name + "/drivePosition", driveEncoder.getPosition());

        Logger.recordOutput("Mod " + name + "/CANCoder_angle", canCoderAngle.in(Degrees));
        Logger.recordOutput("Mod " + name + "/CANCoder_abs_pos", steerCANCoder.getAbsolutePosition().getValue().in(Degrees));
        Logger.recordOutput("Mod " + name + "/normCANCoder", normalizeDegrees(canCoderAngle.in(Degrees)));

        Logger.recordOutput("Mod " + name + "/steerAngle", rawSteerAngle);
        Logger.recordOutput("Mod " + name + "/normSteerAngle", steerAngle);
        Logger.recordOutput("Mod " + name + "/targetSteerAngle", tgtSteerAngle);
        Logger.recordOutput("Mod " + name + "/steerVelocity", steerEncoder.getVelocity());
    }

    public void driveMotorSet(double voltage) {
        driveMotor.set(voltage);
    }

    public void steerMotorSet(double voltage) {
        steerMotor.set(voltage);
    }

    public double getDriveVelocity() {
        return driveVelocity;
    }

    public void setDriveVelocity(LinearVelocity speed) {
        tgtDriveVelocity = speed.in(MetersPerSecond);
        driveController.setReference(tgtDriveVelocity, SparkBase.ControlType.kVelocity);
    }

    public double getSteerAngle() {
        return steerAngle;
    }

    public void setSteerAngle(Angle angle) {
        tgtSteerAngle = angle.in(Degrees);
        steerController.setReference(tgtSteerAngle, SparkBase.ControlType.kPosition);
    }

    public final LinearVelocity getMaxLinearVelocity() {
        // Calculate actual max RPM based on voltage compensation
        double maxRpm = Constants.SparkConstants.FreeSpeedRPM * (Constants.SparkConstants.NominalVoltage / 12.0);

        return MetersPerSecond.of(driveRPMToMetersPerSecond(maxRpm));
    }

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

    private double normalizeDegrees(double unwoundAngleDegrees) {
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

    private void console(String message) {
        System.out.println("SwerveModule (" + name + "): " + message);
    }
}
