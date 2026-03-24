package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Arrays;

import frc.utils.motorcontrol.StormSparkMax;
import frc.utils.motorcontrol.StormTalon;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Shooter;
import frc.robot.RobotState;
import frc.robot.Options.ButtonBoardOptions;
import frc.robot.Options.ShooterOptions;
import frc.utils.StormSubsystem;
import frc.utils.configfile.Option;
import frc.utils.pid.PidConfig;
import org.littletonrobotics.junction.Logger;


public class ShooterHopper extends StormSubsystem {
    public enum ShooterState {
        IDLE,
        SPEEDING_UP,
        SHOOTING,
        DIAGNOSTIC,
        CHARACTERIZING;
    }

    public final StormTalon leaderMotor;
    public final StormTalon followerMotor;
    public final StormSparkMax fuelInjector;

    private final PidConfig pid;
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);
    private final VelocityTorqueCurrentFOC velocityTorqueFOC = new VelocityTorqueCurrentFOC(0).withSlot(0);
    private final boolean useTorqueFOC = Shooter.useTorqueFOC;
    private final NeutralOut neutralOut = new NeutralOut();
    private final VoltageOut sysIdVoltage = new VoltageOut(0);
    private final TorqueCurrentFOC sysIdTorque = new TorqueCurrentFOC(0);
    private final SysIdRoutine sysIdRoutine;
    private double targetVelocityRPS;
    private final ButtonBoardOptions buttonBoardOptions = ButtonBoardOptions.create();
    private final ShooterOptions shooterOptions = ShooterOptions.create();
    private ShooterState state;

    private final InterpolatingDoubleTreeMap shootSpeedMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap funnelSpeedMap = new InterpolatingDoubleTreeMap();
    private boolean useRealDist = false;

    private boolean toHub = true;

    // Flywheel disturbance detection state
    private int     shotCount             = 0;
    private boolean dipActive             = false;
    private boolean detectionArmed        = false;
    private final Timer flywheelIdleTimer = new Timer();
    private final Timer armingTimer       = new Timer();

    @SuppressWarnings("resource")
    public ShooterHopper() {
        if (Shooter.useShooterHopperDiagnostic) {
            state = ShooterState.DIAGNOSTIC;
        } else {
            state = ShooterState.IDLE;
        }

        fuelInjector = new StormSparkMax(Shooter.fuelInjectorMotorID, MotorType.kBrushless).withName("fuelInjector");

        SparkMaxConfig fuelInjectorConfig = new SparkMaxConfig();
        fuelInjectorConfig
            .inverted(true)
            .smartCurrentLimit(Shooter.fuelInjectorCurrentLimitAmps)
            .openLoopRampRate(Shooter.fuelInjectorRampRateOpen);

        fuelInjector.configure(fuelInjectorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // These are on the standard rio CAN, not the canivore
        leaderMotor = new StormTalon(Shooter.leaderMotorID, new CANBus("")).withName("shooterLeader").withClosedLoopLogging();
        followerMotor = new StormTalon(Shooter.followerMotorID, new CANBus("")).withName("shootFollower");

        followerMotor.setControl(
            new Follower(leaderMotor.getDeviceID(), MotorAlignmentValue.Opposed)
        ); // .Opposed
        NeutralModeValue neutralMode = Shooter.coastMode ? NeutralModeValue.Coast : NeutralModeValue.Brake;
        leaderMotor.setNeutralMode(neutralMode);
        followerMotor.setNeutralMode(neutralMode);

        // Initialize PID configs — pick gains based on control mode toggle
        pid = new PidConfig()
                .withKP(useTorqueFOC ? Shooter.shootKP_foc : Shooter.shootKP)
                .withKI(useTorqueFOC ? Shooter.shootKI_foc : Shooter.shootKI)
                .withKD(useTorqueFOC ? Shooter.shootKD_foc : Shooter.shootKD)
                .withKV(useTorqueFOC ? Shooter.shootKV_foc : Shooter.shootKV)
                .withKS(useTorqueFOC ? Shooter.shootKS_foc : Shooter.shootKS)
                .withKA(useTorqueFOC ? Shooter.shootKA_foc : Shooter.shootKA)
                .withKMinMax(Shooter.shootKMin, Shooter.shootKMax);

        // Parse shoot distance/speed lookup from properties into interpolating map
        double[] distances = Arrays.stream(Shooter.shootDistances.replaceAll("[{}]", "").split(","))
                .mapToDouble(Double::parseDouble).toArray();
        double[] speeds = Arrays.stream(Shooter.shootSpeeds.replaceAll("[{}]", "").split(","))
                .mapToDouble(Double::parseDouble).toArray();
        for (int i = 0; i < distances.length; i++) {
            shootSpeedMap.put(distances[i], speeds[i]);
        }

        double[] funnelDistances = Arrays.stream(Shooter.funnelDistances.replaceAll("[{}]", "").split(","))
                .mapToDouble(Double::parseDouble).toArray();
        double[] funnelSpeeds = Arrays.stream(Shooter.funnelSpeeds.replaceAll("[{}]", "").split(","))
                .mapToDouble(Double::parseDouble).toArray();
        for (int i = 0; i < funnelDistances.length; i++) {
            funnelSpeedMap.put(funnelDistances[i], funnelSpeeds[i]);
        }

        // SysId characterization routine — mode-aware (voltage or torque-current)
        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                useTorqueFOC ? Volts.of(20).per(Second) : null,  // amps/s in FOC mode
                useTorqueFOC ? Volts.of(50) : null,              // amps in FOC mode
                null,
                (sysIdState) -> Logger.recordOutput("Shooter/SysIdState", sysIdState.toString())),
            new SysIdRoutine.Mechanism(
                output -> {
                    double value = output.in(Volts); // "volts" but actually amps when useTorqueFOC
                    leaderMotor.setControl(useTorqueFOC
                        ? sysIdTorque.withOutput(value)
                        : sysIdVoltage.withOutput(value));
                },
                log -> {
                    log.motor("shooter")
                        .voltage(Volts.of(leaderMotor.getMotorVoltageOutput()))
                        .current(Amps.of(leaderMotor.getMotorCurrent()))
                        .angularPosition(Rotations.of(leaderMotor.getMotorPosition()))
                        .angularVelocity(RotationsPerSecond.of(leaderMotor.getMotorVelocity()));
                },
                this));

        // Apply gains immediately, then mark dirty so they're re-applied on the first
        // periodic() when CAN is reliably up (constructor call may silently fail).
        configMotor();
        pid.setDirty();
    }

    public void stopShooter() {
        setState(ShooterState.IDLE);
    }

    public boolean isShooting() {
        return state == ShooterState.SPEEDING_UP || state == ShooterState.SHOOTING;
    }

    public void setState(ShooterState newState) {
        if (state == newState) return;
        if (newState == ShooterState.SPEEDING_UP || newState == ShooterState.IDLE) {
            shotCount       = 0;
            dipActive       = false;
            detectionArmed  = false;
            flywheelIdleTimer.reset(); flywheelIdleTimer.stop();
            armingTimer.reset();       armingTimer.stop();
        }
        console("setState: " + state + " -> " + newState, 1);
        state = newState;
    }

    public double getRPS(){
        return (
            toHub
                ? (shootSpeedMap.get(RobotState.getInstance().getDistFromHub()))
                : (funnelSpeedMap.get(RobotState.getInstance().GetDistFromAlliance()))
        );

    }

    public double getTimeOfFlight(double distanceMeters) {
        double rps = toHub ? shootSpeedMap.get(distanceMeters) : funnelSpeedMap.get(distanceMeters);
        double exitVelocity = rps * Math.PI * Shooter.shooterWheelDiameterMeters;
        double horizontalVelocity = exitVelocity * Math.cos(Math.toRadians(Shooter.launchAngleDeg));
        if (horizontalVelocity < 0.1) return 0;
        return distanceMeters / horizontalVelocity;
    }

    public void setToHub(boolean toHub) {
        this.toHub = toHub;
    }

    @Override
    public void periodic() {
        super.periodic();

        fuelInjector.periodic();
        leaderMotor.periodic();
        followerMotor.periodic();

        useRealDist = buttonBoardOptions.usePose.get();

        switch (state) {
            case IDLE -> {
                leaderMotor.setControl(neutralOut);
                fuelInjector.set(0);
            }
            case SPEEDING_UP -> {
                targetVelocityRPS = useRealDist ? getRPS() : shooterOptions.manualRPS.get();
                leaderMotor.setControl(useTorqueFOC
                    ? velocityTorqueFOC.withVelocity(targetVelocityRPS)
                    : velocityVoltage.withVelocity(targetVelocityRPS));
                fuelInjector.set(0);
            }
            case SHOOTING, DIAGNOSTIC -> {
                targetVelocityRPS = useRealDist ? getRPS() : shooterOptions.manualRPS.get();
                leaderMotor.setControl(useTorqueFOC
                    ? velocityTorqueFOC.withVelocity(targetVelocityRPS)
                    : velocityVoltage.withVelocity(targetVelocityRPS));
                fuelInjector.set(shooterOptions.fuelInjectorSpeed.get());

                // Hopper-empty detection (SHOOTING only — not DIAGNOSTIC)
                if (state == ShooterState.SHOOTING) {
                    double wc          = Math.PI * Shooter.shooterWheelDiameterMeters;
                    double setpointMPS = targetVelocityRPS       * wc;
                    double actualMPS   = getCurrentVelocityRPS() * wc;

                    if (!detectionArmed && !armingTimer.isRunning()) {
                        armingTimer.reset(); armingTimer.start();
                    }
                    if (!detectionArmed && armingTimer.hasElapsed(Shooter.hopperEmptyArmedDelaySec)) {
                        detectionArmed = true;
                        flywheelIdleTimer.reset(); flywheelIdleTimer.start();
                    }

                    if (detectionArmed) {
                        boolean dipping = actualMPS < setpointMPS - Shooter.hopperEmptyDipThresholdMPS;
                        if (dipping && !dipActive) {
                            shotCount++;
                            dipActive = true;
                            flywheelIdleTimer.reset();
                        } else if (!dipping) {
                            dipActive = false;
                        }
                    }
                }
            }
            case CHARACTERIZING -> {
                // SysIdRoutine controls the motor directly — just keep fuelInjector off
                fuelInjector.set(0);
            }
        }

        if (pid.isDirty()) {
            configMotor();
            pid.clearDirty();
            console(String.valueOf(pid),1);
        }

        double wc = Math.PI * Shooter.shooterWheelDiameterMeters;
        Logger.recordOutput("Shooter/ShotCount",            shotCount);
        Logger.recordOutput("Shooter/FlywheelBusy",         isFlywheelBusy());
        Logger.recordOutput("Shooter/FlywheelDetectionArmed", detectionArmed);
        Logger.recordOutput("Shooter/FlywheelIdleTimerSec", flywheelIdleTimer.get());
        Logger.recordOutput("Shooter/FlywheelLinearMPS",    getCurrentVelocityRPS() * wc);
        Logger.recordOutput("Shooter/SetpointLinearMPS",    targetVelocityRPS * wc);
    }

    /** Returns true while balls are hitting the flywheel, false when spinning clean. */
    public boolean isFlywheelBusy() {
        if (!detectionArmed) return true;
        return !flywheelIdleTimer.hasElapsed(Shooter.hopperEmptyTimeoutSec);
    }

    public int getShotCount() { return shotCount; }

    public void setUseRealDist(boolean useRealDist) {
        this.useRealDist = useRealDist;
    }

    public PidConfig getPidConfig() {
        return pid;
    }

    void configMotor() {
        boolean ok = true;
        Slot0Configs slot0 = new Slot0Configs();

        slot0.kP = pid.kP;
        slot0.kI = pid.kI;
        slot0.kD = pid.kD;
        slot0.kS = pid.kS;
        slot0.kV = pid.kV;
        slot0.kA = pid.kA;
        ok &= checkApply(leaderMotor.getConfigurator().apply(slot0), "leader slot0");

        MotorOutputConfigs out = new MotorOutputConfigs();
        out.PeakForwardDutyCycle = pid.kMax;
        out.PeakReverseDutyCycle = pid.kMin;
        out.Inverted = InvertedValue.Clockwise_Positive; //used to invert

        MotorOutputConfigs followerOut = new MotorOutputConfigs();
        followerOut.NeutralMode = Shooter.coastMode ? NeutralModeValue.Coast : NeutralModeValue.Brake;

        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit = Shooter.statorCurrentLimit;
        currentLimits.StatorCurrentLimitEnable = true;

        ok &= checkApply(leaderMotor.getConfigurator().apply(currentLimits), "leader currentLimits");
        ok &= checkApply(followerMotor.getConfigurator().apply(currentLimits), "follower currentLimits");

        if (useTorqueFOC) {
            TorqueCurrentConfigs torqueLimits = new TorqueCurrentConfigs();
            torqueLimits.PeakForwardTorqueCurrent = Shooter.statorCurrentLimit;
            torqueLimits.PeakReverseTorqueCurrent = -15;
            ok &= checkApply(leaderMotor.getConfigurator().apply(torqueLimits), "leader torqueLimits");
        }

        ok &= checkApply(followerMotor.getConfigurator().apply(followerOut), "follower motorOutput");
        ok &= checkApply(leaderMotor.getConfigurator().apply(out), "leader motorOutput");

        if (ok) {
            console("configMotor applied successfully: " + pid, 1);
        }
    }
    private boolean checkApply(StatusCode code, String context) {
        if (!code.isOK()) {
            console("configMotor FAILED [" + context + "]: " + code, 1);
            return false;
        }
        return true;
    }

    public double getCurrentVelocityRPS() {
        return leaderMotor.getMotorVelocity();
    }

    public boolean isReadyToFeed() {
        double target = Math.abs(targetVelocityRPS);
        if (target < 1.0) return false;
        double actual = Math.abs(leaderMotor.getMotorVelocity());
        return actual >= target * Shooter.percentTargetReady;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction, Option<Boolean> toggle) {
        return Commands.runOnce(() -> setState(ShooterState.CHARACTERIZING))
            .andThen(sysIdRoutine.quasistatic(direction))
            .finallyDo(() -> { setState(ShooterState.IDLE); toggle.setValue(false); });
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction, Option<Boolean> toggle) {
        return Commands.runOnce(() -> setState(ShooterState.CHARACTERIZING))
            .andThen(sysIdRoutine.dynamic(direction))
            .finallyDo(() -> { setState(ShooterState.IDLE); toggle.setValue(false); });
    }
}
