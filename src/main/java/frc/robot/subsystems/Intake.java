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
        MOVING,
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
    private double rollerSpeed = Constants.Intake.rollerSpeed;

    public boolean hasBeenHomed = false;

    private boolean rumbleHigh = false;
    private final double RUMBLE_MIN = Constants.Intake.RUMBLE_MIN;
    private final double RUMBLE_MAX = Constants.Intake.RUMBLE_MAX;
    private TalonFXConfiguration config;
    private boolean softLimitEnabled = false;

    public Intake() {
        updownIntakeMotor = new TalonFX(Constants.Intake.updownIntakeID, canBusName);
        rollerIntakeMotor = new SparkMax(Constants.Intake.rollerMaxID,MotorType.kBrushless);
        
        config = new TalonFXConfiguration();
        mmRequest = new MotionMagicVoltage(0.0);


        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = Constants.Intake.sensorToMechanismRatio;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        config.Slot0.kP = Constants.Intake.kP; //23.0
        config.Slot0.kI = Constants.Intake.kI; //2.0
        config.Slot0.kD = Constants.Intake.kD; //0.5
        config.Slot0.kG = Constants.Intake.kG;

        config.MotionMagic.MotionMagicCruiseVelocity = Constants.Intake.CruiseVeloc; // rot /s
        config.MotionMagic.MotionMagicAcceleration = Constants.Intake.Accel; // rot/s^2

        updownIntakeMotor.getConfigurator().apply(config);
        updownIntakeMotor.setNeutralMode(NeutralModeValue.Brake);

        updownIntakeMotor.getConfigurator().setPosition(0.0);

        stateTimer = new Timer();
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 130.0 / 360.0; 
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;   

        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0; // might stop it after going down, not sure what to do here?
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        updownIntakeMotor.getConfigurator().apply(config.SoftwareLimitSwitch);
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
                    enableSoftLimit(true);
                    setState(IntakeState.HOMING);

                }
                case HOMING -> { 
                    zeroHere();
                    enableSoftLimit(true);
                    setState(IntakeState.HOME);
                    hasBeenHomed = true;
                }
                // combine these 2 later?
                case HOME -> {
                    enableSoftLimit(true);
                    setAngleDegrees(0);
                    setState(IntakeState.IDLE);
                }
                case IDLE -> {
                    enableSoftLimit(true);
                    setAngleDegrees(1); // Not 0 because it should home at 0 when it's set as far back as possible, this gives some tolerance if pid overshoots. Same w/ kick
                }
                case INTAKING -> {
                    enableSoftLimit(true);
                    setAngleDegrees(110);
                }
                case BUMP -> { // not sure if this is necessary
                    enableSoftLimit(true);
                    setAngleDegrees(60);
                }
                case RUMBLE -> {
                    enableSoftLimit(true);
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
        return Math.abs(getAngleDegrees() - deg) < 10.0; // some tolerance because this method returns a million sig figs
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
        if (hasBeenHomed && state == IntakeState.INTAKING && atTargetDegrees(110)) {
            rollerEnabled = !rollerEnabled;  // toggle on/off
        }
    }
     public void enableSoftLimit(boolean enable){
        if (enable == softLimitEnabled) return; // avoid CAN spam every cycle
        softLimitEnabled = enable;

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 130.0 / 360.0; 
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = enable;   

        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0; // might stop it after going down, not sure what to do here?
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = enable;

        updownIntakeMotor.getConfigurator().apply(config.SoftwareLimitSwitch);
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