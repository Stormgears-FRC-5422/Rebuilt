package frc.robot.subsystems;

import frc.utils.motorcontrol.StormTalon;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Options.IntakeOptions;
import frc.utils.StormSubsystem;
import frc.utils.pid.PidConfig;

public class Intake extends StormSubsystem {

    // ===== Enums =====

    public enum IntakeState {
        UNKNOWN,
        HOMING,
        HOLD,
        PID_MOTION,
        SIMPLE_MOTION,
        IDLE
    }

    public enum MotionProfile {
        IDLE,
        AUTO,
        MANUAL,
    }

    public enum IntakePosition {
        UNKNOWN(Double.NaN),
        HOME(frc.robot.Constants.Intake.homePosition),
        TOP(Double.NEGATIVE_INFINITY),
        // BOTTOM(frc.robot.Constants.Intake.bottomIntakeLimit);
        BOTTOM(Double.POSITIVE_INFINITY);

        private final double position;

        IntakePosition(double position) {
            this.position = position;
        }

        public double getValue() {
            return position;
        }
    }

    // ===== Fields =====

    public final StormTalon updownIntakeMotor;
    private final StormTalon rollerIntakeMotor;
    private final MotionMagicVoltage mmRequest;
    private final NeutralOut neutralRequest = new NeutralOut();
    private final TalonFXConfiguration config = new TalonFXConfiguration();
    private final PidConfig pid;
    private final IntakeOptions intakeOptions = IntakeOptions.create();

    public IntakeState state = IntakeState.UNKNOWN;
    public boolean hasBeenHomed = false;

    private double targetPosition;
    private double currentPosition;
    private double updownVolts;

    private int stallFrameCount = 0;
    private static final int STALL_DEBOUNCE_FRAMES =
        (int) Math.ceil(frc.robot.Constants.Intake.timeBeforeHome / 0.020);

    private boolean rollerEnabled = false;
    private boolean outtakeEnabled = false;
    public double rollerOutput;


    // ===== Constructor =====

    @SuppressWarnings("resource")
    public Intake() {
        updownIntakeMotor = new StormTalon(frc.robot.Constants.Intake.updownIntakeID, new CANBus("")).withName("upDownIntakeMotor").withClosedLoopLogging();
        rollerIntakeMotor = new StormTalon(frc.robot.Constants.Intake.rollerID, new CANBus("")).withName("rollerIntakeMotor");

        CurrentLimitsConfigs rollerCurrentLimits = new CurrentLimitsConfigs();
        rollerCurrentLimits.StatorCurrentLimit = frc.robot.Constants.Intake.rollerStatorCurrentLimit;
        rollerCurrentLimits.StatorCurrentLimitEnable = true;
        rollerCurrentLimits.SupplyCurrentLimit = frc.robot.Constants.Intake.rollerSupplyCurrentLimit;
        rollerCurrentLimits.SupplyCurrentLimitEnable = true;
        rollerIntakeMotor.getConfigurator().apply(rollerCurrentLimits);
        rollerIntakeMotor.setNeutralMode(
            frc.robot.Constants.Intake.rollerCoastMode ? NeutralModeValue.Coast : NeutralModeValue.Brake);

        mmRequest = new MotionMagicVoltage(0.0);

        // PID config for hot-tuning (initial values from properties)
        // Units: output is Volts, error is mechanism rotations (after SensorToMechanismRatio)
        pid = new PidConfig()
                .withKP(frc.robot.Constants.Intake.kP)      // V per mechanism rotation of error
                .withKI(frc.robot.Constants.Intake.kI)      // V per mechanism rotation-second
                .withKD(frc.robot.Constants.Intake.kD)      // V per mechanism rotation/second
                .withKG(frc.robot.Constants.Intake.kG)      // V gravity feedforward (Arm_Cosine)
                .withKS(frc.robot.Constants.Intake.kS);     // V static friction feedforward

        // Motor configuration
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = frc.robot.Constants.Intake.sensorToMechanismRatio;

        // Slot0 PID from config
        config.Slot0.kP = pid.kP;
        config.Slot0.kI = pid.kI;
        config.Slot0.kD = pid.kD;
        config.Slot0.kG = pid.kG;
        config.Slot0.kS = pid.kS;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        // MotionMagic profile constraints (units: mechanism rotations per second)
        config.MotionMagic.MotionMagicCruiseVelocity = frc.robot.Constants.Intake.cruiseVelocity;  // mechanism rot/s
        config.MotionMagic.MotionMagicAcceleration   = frc.robot.Constants.Intake.acceleration;    // mechanism rot/s^2
        config.MotionMagic.MotionMagicJerk           = frc.robot.Constants.Intake.jerk;            // mechanism rot/s^3

        updownIntakeMotor.getConfigurator().apply(config);
        updownIntakeMotor.setNeutralMode(
            frc.robot.Constants.Intake.coastMode ? NeutralModeValue.Coast : NeutralModeValue.Brake);

        // Current limits
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit = frc.robot.Constants.Intake.statorCurrentLimit;
        currentLimits.StatorCurrentLimitEnable = true;
        updownIntakeMotor.getConfigurator().apply(currentLimits);

        targetPosition = 0;
        updownVolts = 0;

        setIdle();
        setState(IntakeState.UNKNOWN);
    }

    // ===== Periodic =====

    @Override
    public void periodic() {
        super.periodic();

        // Refreshes signals and logs Motors/upDownIntakeMotor/* and Motors/rollerIntakeMotor/*
        updownIntakeMotor.periodic();
        rollerIntakeMotor.periodic();

        // Hot-tune PID if changed via dashboard
        if (pid.isDirty()) {
            configMotor();
            pid.clearDirty();
            console(String.valueOf(pid), 1);
        }

        // Read position from cached signal (refreshed by updownIntakeMotor.periodic() above)
        currentPosition = updownIntakeMotor.getMotorPosition() * 360.0;

        // Roller control
        rollerOutput = 0.0;
        if (outtakeEnabled && currentPosition > frc.robot.Constants.Intake.minRollerHeight) {
            rollerOutput = intakeOptions.rollerOuttakeSpeed.get();
        } else if (rollerEnabled && currentPosition > frc.robot.Constants.Intake.minRollerHeight) {
            rollerOutput = -intakeOptions.rollerIntakeSpeed.get();
        }
        rollerIntakeMotor.set(rollerOutput);

        // Arm motor — switch on motion mode only (like elevator)
        switch (state) {
            case HOMING:
                home();
                break;

            case HOLD:
            case PID_MOTION:
                if (hasBeenHomed) {
                    setAngleDegrees(targetPosition);
                } else {
                    console("Stubbornly refusing to move before homed!", 25);
                }
                break;

            case SIMPLE_MOTION:
                if (hasBeenHomed) {
                    simpleMove();
                    updownIntakeMotor.setVoltage(updownVolts);
                } else {
                    console("Stubbornly refusing to move before homed!", 25);
                }
                break;

            default:
                setIdle();
        }

        // Subsystem-level logging (motor signals are under Motors/upDownIntakeMotor/*)
        Logger.recordOutput("Intake/State", state.toString());
        Logger.recordOutput("Intake/EncoderDegrees", currentPosition);
        Logger.recordOutput("Intake/TargetPosition", targetPosition);
        Logger.recordOutput("Intake/RollerOutput", rollerOutput);
    }

    // ===== State Transitions =====

    public void setState(IntakeState newState) {
        state = newState;

        switch (state) {
            case UNKNOWN:
                console("***** UNKNOWN state *****", 1);
                setIdle();
                break;
            case IDLE:
                console("IDLE", 1);
                break;

            case HOMING:
                console("***** HOMING state *****", 1);
                stallFrameCount = 0;
                enableSoftLimits(false);
                break;

            case HOLD:
                console("***** HOLD state *****", 1);
                enableSoftLimits(true);
                break;

            case PID_MOTION:
                console("***** PID_MOTION state *****", 1);
                enableSoftLimits(true);
                break;

            case SIMPLE_MOTION:
                console("***** SIMPLE_MOTION state *****", 1);
                enableSoftLimits(true);
                break;
        }
    }

    // ===== Motor Control Helpers =====

    // Converts degrees to mechanism rotations for MotionMagic position command
    private void setAngleDegrees(double degrees) {
        updownIntakeMotor.setControl(mmRequest.withPosition(degrees / 360.0));
    }

    private void setIdle() {
        updownIntakeMotor.setControl(neutralRequest);
    }

    private void home() {
        if (frc.robot.Constants.Intake.useCurrentLimitHomeStrategy) {
            updownIntakeMotor.setVoltage(-2.0);
        } else {
            updownIntakeMotor.setVoltage(0.0);
        }
    }

    private void simpleMove() {
        if (currentPosition < targetPosition) {
            this.updownVolts = frc.robot.Constants.Intake.updownVolts;
        } else if (currentPosition > targetPosition) {
            this.updownVolts = -frc.robot.Constants.Intake.updownVolts;
        } else {
            this.updownVolts = 0;
        }
    }

    // Applies current PidConfig gains to motor Slot0 (for hot-tuning)
    private void configMotor() {
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = pid.kP;
        slot0.kI = pid.kI;
        slot0.kD = pid.kD;
        slot0.kG = pid.kG;
        slot0.kS = pid.kS;
        slot0.GravityType = GravityTypeValue.Arm_Cosine;
        StatusCode code = updownIntakeMotor.getConfigurator().apply(slot0);
        if (!code.isOK()) console("configMotor FAILED [slot0]: " + code, 1);
    }

    // ===== Position Queries =====

    public double getAngleDegrees() {
        return updownIntakeMotor.getMotorPosition() * 360.0;
    }

    public boolean isAtHome() {
        if (frc.robot.Constants.Intake.useCurrentLimitHomeStrategy) {
            if (updownIntakeMotor.getMotorCurrent() > frc.robot.Constants.Intake.stallCurrentLimit) {
                stallFrameCount++;
            } else {
                stallFrameCount = 0;
            }
            return stallFrameCount >= STALL_DEBOUNCE_FRAMES;
        } else {
            console("isAtHome current position: " + currentPosition);
            return true;
        }
    }

    public boolean isAtTarget() {
        return Math.abs(currentPosition - targetPosition) < frc.robot.Constants.Intake.targetDegreesTolerance;
    }

    // ===== Public API =====

    public PidConfig getPidConfig() {
        return pid;
    }

    public void declareHome() {
        hasBeenHomed = true;
        updownIntakeMotor.getConfigurator().setPosition(-0.00444, 0.250);
        updownIntakeMotor.set(0);
        setIdle();
        currentPosition = -1.6;
        console("HOMED!!!!",1);
    }

    public void setTargetPosition(IntakePosition position) {
        setTargetPosition(position.getValue());
    }

    public void setTargetPosition(double degrees) {
        this.targetPosition = degrees;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public void setRollerEnabled(boolean enabled) {
        rollerEnabled = enabled;
    }

    public void setOuttakeRollers(boolean enabled) {
        outtakeEnabled = enabled;
    }

    // Soft limits (thresholds in mechanism rotations, converted from degrees)
    public void enableSoftLimits(boolean enable) {
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = frc.robot.Constants.Intake.bottomIntakeLimit / 360.0;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = enable;

        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = frc.robot.Constants.Intake.topIntakeLimit / 360.0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = enable;

        updownIntakeMotor.getConfigurator().apply(config.SoftwareLimitSwitch);
    }

    public void holdCurrentPosition() {
        setTargetPosition(currentPosition);
        setState(IntakeState.HOLD);
        setAngleDegrees(currentPosition);
    }

    public void holdCurrentPosition(double position) {
        setTargetPosition(position);
        setState(IntakeState.HOLD);
        setAngleDegrees(position);
    }

    public void setMotionProfile(MotionProfile profile) {
        if (profile == MotionProfile.IDLE) {
            setIdle();
            return;
        }
        MotionMagicConfigs mm = new MotionMagicConfigs();
        if (profile == MotionProfile.MANUAL) {
            mm.MotionMagicCruiseVelocity = frc.robot.Constants.Intake.manualCruiseVelocity;
            mm.MotionMagicAcceleration   = frc.robot.Constants.Intake.manualAcceleration;
            mm.MotionMagicJerk           = frc.robot.Constants.Intake.manualJerk;
        } else {
            mm.MotionMagicCruiseVelocity = frc.robot.Constants.Intake.cruiseVelocity;
            mm.MotionMagicAcceleration   = frc.robot.Constants.Intake.acceleration;
            mm.MotionMagicJerk           = frc.robot.Constants.Intake.jerk;
        }
        updownIntakeMotor.getConfigurator().apply(mm);
    }

    public double getCurrentPosition() {
        return currentPosition;
    }
}
