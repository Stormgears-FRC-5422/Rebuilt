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
    
    double current;
    public IntakeState state = IntakeState.UNKNOWN;
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

        config.Slot0.kP = 60.0; //23.0
        config.Slot0.kI = 1.0; //2.0
        config.Slot0.kD = 0.0; //0.5
        config.Slot0.kG = 0.0;

        config.MotionMagic.MotionMagicCruiseVelocity = 5.0; // rot /s
        config.MotionMagic.MotionMagicAcceleration = 5.0; // rot/s^2

        updownIntakeMotor.getConfigurator().apply(config);
        updownIntakeMotor.setNeutralMode(NeutralModeValue.Coast);

        stateTimer = new Timer();

        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit = 40.0;
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

    public void setState(IntakeState newState) {
        state = newState;
    }

    public boolean atTargetDegrees(double deg) {
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
            rumblePin = Shooter.minRumbleAngle;
            rumbleHigh = false;
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
            setState(IntakeState.INTAKING);
        }
    }
    @Override
    public void periodic() {
        super.periodic();

        current = updownIntakeMotor.getSupplyCurrent().getValueAsDouble();

        Logger.recordOutput("rumble target", rumblePin);
        Logger.recordOutput("rumble position", getAngleDegrees());
        Logger.recordOutput("rumble at target", atTargetDegrees(rumblePin));

        Logger.recordOutput("intake motor up down current", updownIntakeMotor.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("intake stator current", updownIntakeMotor.getStatorCurrent().getValueAsDouble());
        if (rollerEnabled && state == IntakeState.INTAKING) {
            // System.out.println("Intake + " + elasticMotorController.intakeRollerSpeed.get());
            rollerIntakeMotor.set(Shooter.rollerIntakeSpeed);
        } else {
            rollerIntakeMotor.set(0.0);
        }

        switch (state) {
            case UNKNOWN -> { //insta go home
                // setState(IntakeState.HOMING);
                updownIntakeMotor.set(0);
                rollerIntakeMotor.set(0);
            }
            case HOMING -> {
                home();
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
    private void home() {
        if (Shooter.stratlimit) {
            // might want to use an average here to minimize spikes
            updownIntakeMotor.setVoltage(-2.0);
        } else { //assume you're at home
            updownIntakeMotor.setVoltage(0.0);
        }
    }    
    public boolean isAtHome() {
        if (Shooter.stratlimit) {
            // might want to use an average here to minimize spikes
            return (updownIntakeMotor.getStatorCurrent().getValueAsDouble()) > Shooter.intakeCurrentLimit;
        } else {
            console("isAtHome current position: " + updownIntakeMotor.getPosition());
            return true;
        }
    }
}
