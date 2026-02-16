package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.Shooter;
import frc.utils.StormSubsystem;
import frc.utils.pid.PidConfig;

public class ShooterHopper extends StormSubsystem {
    public enum ShooterState {
        IDLE, //default state
        SPEEDING_UP, // time that it takes to reach the speed, may eventually be on a 1 second timer
        SHOOTING, // hopper rollers should be running, probably also run the intake to get any balls that it's touching
        OUTTAKE, // sends the ball back to the hopper
        EJECT, //outtakes slowly out of the robot 
        BAD;
    }
    private final Timer spinupTimer = new Timer();
    private static final double spinuptimeseconds = 1.0;

    public final TalonFX leaderMotor;
    public final TalonFX followerMotor;
    public final SparkMax hopperNeo;

    private final PidConfig pid;
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);
    private double targetVelocityRPS;
    private ShooterState state = ShooterState.IDLE;
    private double hopperSpeed = 0.0;
    int num = 0;

    public ShooterHopper() {
        leaderMotor = new TalonFX(Shooter.leaderMotorID);
        followerMotor = new TalonFX(Shooter.followerMotorID);
        hopperNeo = new SparkMax(Shooter.hopperMotorID, MotorType.kBrushless);

        // followerMotor.setControl(
        //     new Follower(leaderMotor.getDeviceID(), MotorAlignmentValue.Opposed)
        // ); // .Opposed 

        // Initialize PID configs
        pid = new PidConfig()
                .withKP(Shooter.shootKP)
                .withKI(Shooter.shootKI)
                .withKD(Shooter.shootKD)
                .withKV(Shooter.shootKV)
                .withKS(Shooter.shootKS)
                .withKMinMax(Shooter.shootKMin, Shooter.shootKMax);

        configMotor();
    }

    public void setTargetVelocity(double givenTarget) {
        if (state == ShooterState.SPEEDING_UP || state == ShooterState.SHOOTING) {
            targetVelocityRPS = givenTarget;
        }
    }
    
    public void stopShooter() {
        setState(ShooterState.IDLE);
        targetVelocityRPS = 0.0;
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
                targetVelocityRPS = Shooter.testingTargetRPS;
                hopperSpeed = 0.0;
            }
            case SHOOTING -> {
                targetVelocityRPS = Shooter.testingTargetRPS;
                hopperSpeed = Shooter.hopperSpeed;
            }
            case OUTTAKE -> {
                targetVelocityRPS = -10.0;
            }
            case EJECT -> {
                targetVelocityRPS = 5.0;
            }
            case BAD -> {}
        }
    }

    @Override
    public void periodic() {
        super.periodic();
            
        shooterStateMachine(); 
        leaderMotor.setControl(velocityVoltage.withVelocity(targetVelocityRPS));
        hopperNeo.set(hopperSpeed);

        if (pid.isDirty()) {
            configMotor();
            pid.clearDirty();
            console(String.valueOf(pid),1);
        }
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

        //follower coast config, for testing 1 motor at a time
        MotorOutputConfigs followerOut = new MotorOutputConfigs();
        followerOut.NeutralMode = NeutralModeValue.Coast;

        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit = 60.0; //amps
        currentLimits.StatorCurrentLimitEnable = true;

        leaderMotor.getConfigurator().apply(currentLimits);

        followerMotor.getConfigurator().apply(followerOut);
        leaderMotor.getConfigurator().apply(out);
    }
}