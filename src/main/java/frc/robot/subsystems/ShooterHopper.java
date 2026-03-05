package frc.robot.subsystems;

import java.util.Arrays;

import frc.utils.motorcontrol.StormSparkMax;
import frc.utils.motorcontrol.StormTalon;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.Shooter;
import frc.robot.RobotState;
import frc.robot.elastic.ElasticMotorController;
import frc.utils.StormSubsystem;
import frc.utils.pid.PidConfig;


public class ShooterHopper extends StormSubsystem {
    public enum ShooterState {
        IDLE, //default state
        SPEEDING_UP, // time that it takes to reach the speed, may eventually be on a 1 second timer
        SHOOTING, // hopper rollers should be running, probably also run the intake to get any balls that it's touching
        OUTTAKE, // sends the ball back to the hopper
        EJECT, //outtakes slowly out of the robot
        BAD,
        DIAGNOSTIC;
    }

    public final StormTalon leaderMotor;
    public final StormTalon followerMotor;
    public final StormSparkMax hopperNeo;

    private final PidConfig pid;
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);
    private double targetVelocityRPS;
    private double hopperSpeed = 0.0;
    private final ElasticMotorController elasticMotorController;
    private ShooterState state;

    private final InterpolatingDoubleTreeMap shootSpeedMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap funnelSpeedMap = new InterpolatingDoubleTreeMap();
    private boolean useRealDist = false;

    private boolean toHub = true;

    // Cached CAN status signal — refreshed in a single batch each cycle
    private final StatusSignal<AngularVelocity> rotorVelocitySignal;


    public ShooterHopper(ElasticMotorController elasticMotorController) {
        this.elasticMotorController = elasticMotorController;

        if (Shooter.useShooterHopperDiagnostic) {
            state = ShooterState.DIAGNOSTIC;
        } else {
            state = ShooterState.IDLE;
        }

        hopperNeo = new StormSparkMax(Shooter.hopperMotorID, MotorType.kBrushless).withName("hopperMotor");

        SparkMaxConfig hopperConfig = new SparkMaxConfig();
        hopperConfig
            .inverted(true)
            .smartCurrentLimit(Shooter.hopperCurrentLimitAmps)
            .openLoopRampRate(Shooter.hopperRampRateOpen);

        hopperNeo.configure(hopperConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // These are on the standard rio CAN, not the canivore
        leaderMotor = new StormTalon(Shooter.leaderMotorID, new CANBus("")).withName("shooterLeader");
        followerMotor = new StormTalon(Shooter.followerMotorID, new CANBus("")).withName("shootFollower");

        followerMotor.setControl(
            new Follower(leaderMotor.getDeviceID(), MotorAlignmentValue.Opposed)
        ); // .Opposed
        NeutralModeValue neutralMode = Shooter.coastMode ? NeutralModeValue.Coast : NeutralModeValue.Brake;
        leaderMotor.setNeutralMode(neutralMode);
        followerMotor.setNeutralMode(neutralMode);

        // Initialize PID configs
        pid = new PidConfig()
                .withKP(Shooter.shootKP)
                .withKI(Shooter.shootKI)
                .withKD(Shooter.shootKD)
                .withKV(Shooter.shootKV)
                .withKS(Shooter.shootKS)
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

        // Cache status signal for batched CAN reads
        rotorVelocitySignal = leaderMotor.getRotorVelocity();

        configMotor();
    }

    public void setTargetVelocity(double givenTarget) {
        if (state == ShooterState.SPEEDING_UP || state == ShooterState.SHOOTING) {
            targetVelocityRPS = givenTarget;
        }
    }

    public void stopShooter() {
        setState(ShooterState.IDLE);
    }

    public void setState(ShooterState newState) {
        if (state == newState) return;
        state = newState;
    }

    public void shooterStateMachine() {
        switch (state) {
            case IDLE -> {
                targetVelocityRPS = 0.0;
                hopperSpeed = 0.0;
            }
            case SPEEDING_UP -> {
                targetVelocityRPS =
                    useRealDist
                        ? getRPS()
                        : (elasticMotorController.shootSpeed.get() * 100);
                hopperSpeed = 0.0;
            }
            case SHOOTING -> {
                targetVelocityRPS =
                    useRealDist
                        ? getRPS()
                        : (elasticMotorController.shootSpeed.get() * 100);
                hopperSpeed = Shooter.hopperSpeed;
            }
            case OUTTAKE -> {
                targetVelocityRPS = -10.0;
                hopperSpeed = -0.5;
            }
            case EJECT -> {
                targetVelocityRPS = 5.0;
            }
            case BAD -> {}
            case DIAGNOSTIC -> {
                targetVelocityRPS = 0;
                hopperSpeed = 0;
            }
        }
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

        hopperNeo.logMotorData();
        leaderMotor.logMotorData();
        followerMotor.logMotorData();

        // Refresh cached signal (non-blocking — reads from local cache)
        BaseStatusSignal.refreshAll(rotorVelocitySignal);

        useRealDist = elasticMotorController.usePose.get();

        shooterStateMachine();
        if (state == ShooterState.DIAGNOSTIC) {
            leaderMotor.setControl(velocityVoltage.withVelocity(100 * elasticMotorController.shootSpeed.get()));
            hopperNeo.set(elasticMotorController.hopperSpeed.get());
        } else {
            leaderMotor.setControl(velocityVoltage.withVelocity(targetVelocityRPS));
            hopperNeo.set(hopperSpeed);
        }

        if (pid.isDirty()) {
            configMotor();
            pid.clearDirty();
            console(String.valueOf(pid),1);
        }
        Logger.recordOutput("shooter rps", rotorVelocitySignal.getValueAsDouble());
    }

    public void setUseRealDist(boolean useRealDist) {
        this.useRealDist = useRealDist;
    }

    public PidConfig getPidConfig() {
        return pid;
    }

    void configMotor() {
        Slot0Configs slot0 = new Slot0Configs();

        // From phoenix 6 examples
        // Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
        // configs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
        // configs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
        // configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
        // configs.Slot0.kI = 0; // No output for integrated error
        // configs.Slot0.kD = 0; // No output for error derivative

        slot0.kP = pid.kP;
        slot0.kI = pid.kI;
        slot0.kD = pid.kD;
        slot0.kS = pid.kS;
        slot0.kV = pid.kV;
        leaderMotor.getConfigurator().apply(slot0);

        MotorOutputConfigs out = new MotorOutputConfigs();
        out.PeakForwardDutyCycle = pid.kMax;
        out.PeakReverseDutyCycle = pid.kMin;
        out.Inverted = InvertedValue.Clockwise_Positive; //used to invert

        MotorOutputConfigs followerOut = new MotorOutputConfigs();
        followerOut.NeutralMode = Shooter.coastMode ? NeutralModeValue.Coast : NeutralModeValue.Brake;

        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit = Shooter.statorCurrentLimit;
        currentLimits.StatorCurrentLimitEnable = true;

        leaderMotor.getConfigurator().apply(currentLimits);
        followerMotor.getConfigurator().apply(currentLimits);

        followerMotor.getConfigurator().apply(followerOut);
        leaderMotor.getConfigurator().apply(out);
    }
    public double getCurrentVelocityRPS() {
        return rotorVelocitySignal.getValueAsDouble();
    }

    public double getTargetVelocityRPS() {
        return targetVelocityRPS;
    }

    public boolean isReadyToFeed() {
        double target = Math.abs(targetVelocityRPS);
        if (target < 1.0) return false;
        double actual = Math.abs(rotorVelocitySignal.getValueAsDouble());
        return actual >= target * Shooter.percentTargetReady;
    }
}
