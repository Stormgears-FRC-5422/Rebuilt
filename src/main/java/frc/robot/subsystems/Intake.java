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
        PID_MOTION,
        SIMPLE_MOTION,
    }

    public enum IntakePositions {
        UNKNOWN(Double.NaN),
        NEG_INFINITY(Double.NEGATIVE_INFINITY),
        HOME(0),
        TOP(Shooter.topIntakeLimit),
        BOTTOM(Shooter.bottomIntakeLimit),
        POS_INFINITY(Double.POSITIVE_INFINITY);

        private double position;

        IntakePositions(double position) {
            this.position = position;
        }

        public double getValue() {
            return position;
        }
    }

    private double target;
    private final TalonFXConfiguration config = new TalonFXConfiguration();

    public ElasticMotorController elasticMotorController = new ElasticMotorController();

    public final TalonFX updownIntakeMotor;
    private final SparkMax rollerIntakeMotor;

    private final MotionMagicVoltage mmRequest;

    double current;
    public IntakeState state = IntakeState.UNKNOWN;

    String canBusName = Constants.Imu.canBusName;

    private boolean rollerEnabled = false;
    private boolean outtakeEnabled = false;
    public boolean hasBeenHomed = false;
    private boolean softLimitEnabled = false;
    public double rollerOutput;

    //rumble stuff
    private double rumbleMinAngle = Shooter.minRumbleAngle;
    private double rumbleMaxAngle = Shooter.maxRumbleAngle;
    private boolean rumbleHitMax = false;
    private boolean rumbleHigh = false;

    private double rumblePin = 40.0;
    private double updownIntakeMotorSpeed = 0.0;
    private double rollerIntakeMotorSpeed = 0.0;

    private double updownVolts;
    private double targetPosition;
    private double currentPosition;

    public Intake() {
        updownIntakeMotor = new TalonFX(Shooter.upDownMotorID, new CANBus(""));
        rollerIntakeMotor = new SparkMax(Shooter.rollerMotorID,MotorType.kBrushless);

        mmRequest = new MotionMagicVoltage(0.0);

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = 53.33;

        config.Slot0.kP = 80.0; //23.0
        config.Slot0.kI = 2.0; //2.0
        config.Slot0.kD = 0.3; //0.5
        config.Slot0.kG = 0.0; // probably need this

        config.Slot0.kS = 0.20;  // static friction  V

        config.MotionMagic.MotionMagicCruiseVelocity = 5.0;  // rot/s
        config.MotionMagic.MotionMagicAcceleration   = 3.0;  // rot/s^2
        config.MotionMagic.MotionMagicJerk           = 30; // rot/s^3

        updownIntakeMotor.getConfigurator().apply(config);
        updownIntakeMotor.setNeutralMode(NeutralModeValue.Coast);

        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit = 35.0;
        currentLimits.StatorCurrentLimitEnable = true;

        updownIntakeMotor.getConfigurator().apply(currentLimits);

        targetPosition = IntakePositions.UNKNOWN.position;
        updownVolts = 0;
    }

    public void setAngleDegrees(double degrees) {
        double accRotations = degrees / 360.0;

        updownIntakeMotor.setControl(mmRequest.withPosition(accRotations));
    }

    public void zeroHere() {
        updownIntakeMotor.getConfigurator().setPosition(0.0);
        enableSoftLimits(true);
        hasBeenHomed = true;
    }

    public double getAngleDegrees() {
        return updownIntakeMotor.getPosition().getValue().in(Degrees);
    }

    public void setState(IntakeState newState) {
        state = newState;
        
        switch (state) {
            case UNKNOWN -> {
                console("***** UNKNOWN state *****");
            }
            case HOMING -> {
                console("***** HOMING state *****");
                enableSoftLimits(false);
            }
            case IDLE -> {
                console("***** IDLE state *****");
                setAngleDegrees(Shooter.topIntakeDegrees);
            }
            case INTAKING -> {
                console("***** INTAKING state *****");
                setAngleDegrees((elasticMotorController.intakeAngle.get().doubleValue()));
            }
            case RUMBLE -> {
            }
        }

    }

    public boolean atTargetDegrees(double deg) {
        return Math.abs(getAngleDegrees() - deg) < Shooter.targetDegreesTolerance;
    }

    public void requestRumble() {
        if (hasBeenHomed) {
            rumbleMinAngle = Shooter.minRumbleAngle;
            rumbleMaxAngle = Shooter.maxRumbleAngle;
            rumblePin = rumbleMinAngle;
            rumbleHigh = false;
            rumbleHitMax = false;
            setState(IntakeState.RUMBLE);
        }
    }

    public void requestIntakeDown() {
        if (hasBeenHomed) {
            setState(IntakeState.INTAKING);
        }
    }
    public void requestIdle() {
        if (hasBeenHomed) {
            setState(IntakeState.IDLE);
        }
    }
    public void setRollerEnabled(boolean enabled) {
        rollerEnabled = enabled;
    }
    public void setOuttakeRollers(boolean enabled) {
        outtakeEnabled = enabled;
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
        currentPosition = updownIntakeMotor.getPosition().getValueAsDouble();

        current = updownIntakeMotor.getSupplyCurrent().getValueAsDouble();

        Logger.recordOutput("rumble target", rumblePin);
        Logger.recordOutput("rumble position", getAngleDegrees());
        Logger.recordOutput("rumble at target", atTargetDegrees(rumblePin));

        Logger.recordOutput("intake motor up down current", updownIntakeMotor.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("intake stator current", updownIntakeMotor.getStatorCurrent().getValueAsDouble());

        Logger.recordOutput("intake encoder degrees", getAngleDegrees());
        Logger.recordOutput("intake raw encoder rotations", updownIntakeMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("intake target angle", elasticMotorController.intakeAngle.get().doubleValue());
        Logger.recordOutput("intake at target", atTargetDegrees(elasticMotorController.intakeAngle.get().doubleValue()));
        Logger.recordOutput("intake state", state.toString());
        Logger.recordOutput("intake voltage", updownIntakeMotor.getMotorVoltage().getValueAsDouble());

        rollerOutput = 0.0;
        if (outtakeEnabled) {
            rollerOutput = Shooter.rollerOuttakeSpeed;
        } else if (rollerEnabled && state == IntakeState.INTAKING) {
            rollerOutput = Shooter.rollerIntakeSpeed;
        }
        rollerIntakeMotor.set(rollerOutput);

        switch (state) {
            case UNKNOWN -> {
                updownIntakeMotor.set(0);
            }
            // case HOMING -> {
            //     home();
            // }
            // case IDLE -> {
            //     // setAngleDegrees(Shooter.topIntakeDegrees);
            // }
            // case INTAKING -> {
            //     setAngleDegrees(elasticMotorController.intakeAngle.get().doubleValue());
            // }
            // case RUMBLE -> {
            //     setAngleDegrees(rumblePin);

            //     if (atTargetDegrees(rumblePin)) {
            //         if (rumbleHigh) {
            //             rumbleHitMax = true;
            //         }

            //         rumbleHigh = !rumbleHigh;
            //         rumblePin = rumbleHigh ? rumbleMaxAngle : rumbleMinAngle;
            //     }
            // }
            case PID_MOTION -> {
                setAngleDegrees(target);
            }
            case SIMPLE_MOTION -> {
                if (hasBeenHomed) {
                    simpleMove(currentPosition);
                    updownIntakeMotor.setVoltage(updownVolts);
                } else {
                    console("Stubbornly refusing to move before homed!", 25);
                }
            }
        }
    }
    private void setTarget(double target) {
        this.target = target;
    }
    private void setSimpleMotionVolts(double volts) {
        volts = updownVolts;
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

    public void setRumbleMinAngle(double degrees) {
        rumbleMinAngle = degrees;
    }

    public boolean intakeIsAtMax() {
        boolean hit = rumbleHitMax;
        rumbleHitMax = false;
        return hit;
    }
    
    public void enableSoftLimits(boolean enable) {
        softLimitEnabled = enable;

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Shooter.bottomIntakeLimit / 360.0;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = enable;

        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Shooter.topIntakeLimit / 360.0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = enable;

        updownIntakeMotor.getConfigurator().apply(config.SoftwareLimitSwitch);
    }


    public void requestPidMotion(double degrees) {
        if (!hasBeenHomed) return;

        setTarget(degrees);
        setState(IntakeState.PID_MOTION);
    }

    private void simpleMove(double currentPosition) {
        if (currentPosition < targetPosition) {
            setSimpleMotionVolts(Shooter.updownVolts);
        } else if (currentPosition > targetPosition) {
            setSimpleMotionVolts(-updownVolts);
        } else {
            setSimpleMotionVolts(0);
        }
    }
    public void setTargetPosition(IntakePositions targetPosition) {
        setTargetPosition(targetPosition.getValue());
    }
    public void setTargetPosition(double position) {
        this.targetPosition = position;
    }
}
