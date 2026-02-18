package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.Shooter;
import frc.robot.elastic.ElasticMotorController;
import frc.utils.StormSubsystem;


public class Intake extends StormSubsystem {
    public enum IntakeState {
        UNKNOWN,
        HOMING, 
        IDLE, // Once homing has homed the intake, it is sent to IDLE. also brings intake arm back up, leaving it there
        INTAKING, // in the position to intake
        RUMBLE, // rumbles to funnel balls into shooter
    }
    public ElasticMotorController elasticMotorController = new ElasticMotorController();

    public final TalonFX updownIntakeMotor;
    private final SparkMax rollerIntakeMotor;

    private final MotionMagicVoltage mmRequest;

    private IntakeState state = IntakeState.UNKNOWN;
    private final Timer stateTimer;
    String canBusName = Constants.Imu.canBusName;

    private boolean rollerEnabled = false;
    public boolean hasBeenHomed = false;
    private boolean rumbleHigh = false;

    private double rumblePin = 40.0;
    public Intake() {
        updownIntakeMotor = new TalonFX(Shooter.upDownMotorID, new CANBus(""));
        rollerIntakeMotor = new SparkMax(Shooter.rollerMotorID,MotorType.kBrushless);
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        mmRequest = new MotionMagicVoltage(0.0);

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = 53.33;

        config.Slot0.kP = 50.0; //23.0
        config.Slot0.kI = 0.0; //2.0
        config.Slot0.kD = 0.0; //0.5
        config.Slot0.kG = 0.0;

        config.MotionMagic.MotionMagicCruiseVelocity = 1.0; // rot /s
        config.MotionMagic.MotionMagicAcceleration = 1.0; // rot/s^2

        updownIntakeMotor.getConfigurator().apply(config);
        updownIntakeMotor.setNeutralMode(NeutralModeValue.Coast);

        stateTimer = new Timer();

        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit = 20.0;
        currentLimits.StatorCurrentLimitEnable = true;

        updownIntakeMotor.getConfigurator().apply(currentLimits);
    }

    public void setAngleDegrees(double degrees) {
        double accRotations = degrees / 360.0;

        updownIntakeMotor.setControl(mmRequest.withPosition(accRotations));
    }

    public void zeroHere() {
        updownIntakeMotor.getConfigurator().setPosition(0.0);
    }

    public double getAngleDegrees() {
        return updownIntakeMotor.getPosition().getValue().in(Degrees);
    }
    
    private void setState(IntakeState newState) {
        state = newState;
    }
    
    private boolean atTargetDegrees(double deg) {
        return Math.abs(getAngleDegrees() - deg) < Shooter.targetDegreesTolerance; 
    }
    
    // only methods to be used outside
    public void requestHome() {
        setState(IntakeState.HOMING);
    }
    public void requestIdle() {
        if (hasBeenHomed) {
            setState(IntakeState.IDLE);
        }
    }
    public void requestRumble() {
        if (hasBeenHomed) {
        setState(IntakeState.RUMBLE);
        }
    }
    public void requestIntakeDown() {
        if (hasBeenHomed) {
            setState(IntakeState.INTAKING);
        }
    }
    public void setRollerEnabled(boolean enabled) {
        rollerEnabled = enabled;
    }
    public void toggleIntakePosition() {
        if (!hasBeenHomed) return;

        if (state == IntakeState.INTAKING) {
            rollerEnabled = false;
            setState(IntakeState.IDLE);
        } else {
            setState(IntakeState.INTAKING)
        }
    }
    @Override
    public void periodic() {        
        super.periodic();
        
        Logger.recordOutput("intake motor up down current", updownIntakeMotor.getSupplyCurrent().getValueAsDouble());

        if (rollerEnabled && state == IntakeState.INTAKING && atTargetDegrees(Shooter.intakingAngle)) {
            rollerIntakeMotor.set(elasticMotorController.intakeRollerSpeed.get());
        } else {
            rollerIntakeMotor.set(0.0);
        }

        switch (state) {
            case UNKNOWN -> { //insta go home
                setState(IntakeState.HOMING);
            }
            case HOMING -> { 
                zeroHere();
                setState(IntakeState.IDLE);
                hasBeenHomed = true;
                // updownIntakeMotor.set(Shooter.homingSpeed);
                // double current = updownIntakeMotor.getSupplyCurrent().getValueAsDouble();
                // //need to log current to find the amount to home at
                // if (current > 15.0) { // need to experiment to find the right current threshold that indicates stalling against the hard stop
                //         if (stateTimer.hasElapsed(0.1)) {
                //             updownIntakeMotor.stopMotor();
                //             zeroHere();
                //             hasBeenHomed = true;
                //             setState(IntakeState.IDLE);
                //     }
            }
            case IDLE -> {
                setAngleDegrees(1);
            }
            case INTAKING -> {
                setAngleDegrees(Shooter.intakingAngle);
            }
            case RUMBLE -> {
                setAngleDegrees(rumblePin);

                if (atTargetDegrees(rumblePin)) {
                    if (rumbleHigh) {
                        rumblePin = Shooter.maxRumbleAngle;
                    } else {
                        rumblePin = Shooter.minRumbleAngle;
                    }
                    rumbleHigh = !rumbleHigh;
                }
                            
            }
        }
    }
}