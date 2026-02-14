package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.utils.StormSubsystem;

public class Intake extends StormSubsystem {
    public enum IntakeState {
        UNKNOWN,
        HOMING,
        HOME,
        IDLE, // brings intake arm back up, leaving it there
        INTAKING, // in the position to intake
        RUMBLE, // rumbles to funnel
        BUMP,
    }

    public final TalonFX updownIntakeMotor;
    private final SparkMax rollerIntakeMotor;
    private final MotionMagicVoltage mmRequest;

    private double targetDeg = 0.0;


    private IntakeState state = IntakeState.UNKNOWN;
    private IntakeState lastState = null;
    private final Timer stateTimer;
    String canBusName = Constants.Imu.canBusName;

    private boolean rollerEnabled = false;
    private double rollerSpeed = 0.8;

    public boolean hasBeenHomed = false;

    private boolean rumbleHigh = false;
    private final double RUMBLE_MIN = 40.0;
    private final double RUMBLE_MAX = 50.0;

    public Intake() {
        updownIntakeMotor = new TalonFX(25, canBusName);
        rollerIntakeMotor = new SparkMax(51,MotorType.kBrushless);
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        mmRequest = new MotionMagicVoltage(0.0);

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = 53.33;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        config.Slot0.kP = 50.0; //23.0
        config.Slot0.kI = 0.0; //2.0
        config.Slot0.kD = 0.0; //0.5
        config.Slot0.kG = 0.0;

        config.MotionMagic.MotionMagicCruiseVelocity = 0.2; // rot /s
        config.MotionMagic.MotionMagicAcceleration = 0.2; // rot/s^2

        updownIntakeMotor.getConfigurator().apply(config);
        updownIntakeMotor.setNeutralMode(NeutralModeValue.Brake);

        updownIntakeMotor.getConfigurator().setPosition(0.0);

        stateTimer = new Timer();
    }

    public void setAngleDegrees(double degrees) {
        targetDeg = degrees;
        double accRotations = degrees / 360.0;

        updownIntakeMotor.setControl(mmRequest.withPosition(accRotations));
    }

    public void zeroHere() {
        updownIntakeMotor.getConfigurator().setPosition(0.0);
        targetDeg = 0.0;
    }

    public double getAngleDegrees() {
        return updownIntakeMotor.getPosition().getValue().in(Degrees);
    }
    
    private void setState(IntakeState newState) {
        state = newState;
    }

    private void updateStateMachine() {
        if (state != lastState) {
            stateTimer.restart();
            lastState = state;

            // if (state == IntakeState.INTAKING && rollerEnabled)
            switch (state) {
                case UNKNOWN -> { //insta go home
                    setState(IntakeState.HOMING);
                }
                case HOMING -> { 
                    zeroHere();
                    setState(IntakeState.HOME);
                    hasBeenHomed = true;
                }
                // combine these 2 later?
                case HOME -> {
                    setAngleDegrees(0);
                    setState(IntakeState.IDLE);
                }
                case IDLE -> {
                    setAngleDegrees(1); // Not 0 because it should home at 0 when it's set as far back as possible, this gives some tolerance if pid overshoots. Same w/ kick
                }
                case INTAKING -> {
                    setAngleDegrees(110);
                }
                case BUMP -> { // not sure if this is necessary
                    setAngleDegrees(60);
                }
                case RUMBLE -> {
                    rumbleHigh = false;
                    setAngleDegrees(RUMBLE_MIN);
                }
            }
            if (state == IntakeState.RUMBLE) {
                if (stateTimer.hasElapsed(0.5)) {
                    stateTimer.restart();

                    rumbleHigh = !rumbleHigh;

                    if (rumbleHigh) {
                        setAngleDegrees(RUMBLE_MAX);
                    } else {
                        setAngleDegrees(RUMBLE_MIN);
                    }
                }
            }

        }
    }
    
    private boolean atTargetDegrees(double deg) {
        return Math.abs(getAngleDegrees() - deg) < 1.0; // some tolerance because this method returns a million sig figs
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
    public void runIntake() {
        // add a secondary if later on, in case it needs to be disabled even when it isnt in intaking
        if (hasBeenHomed && state == IntakeState.INTAKING) {
            rollerEnabled = !rollerEnabled;  // toggle on/off
        }
    }

    @Override
    public void periodic() {
        updateStateMachine();
        if (rollerEnabled && state == IntakeState.INTAKING) {
            rollerIntakeMotor.set(rollerSpeed);
        } else {
            rollerIntakeMotor.set(0.0);
        }
    }
}