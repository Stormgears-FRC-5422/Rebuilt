package frc.robot.subsystems.ballControl;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
        KICK, // brings intake up to top, then sends it back down to intaking
    }

    private final TalonFX updownIntakeMotor;
    private final TalonFX rollerIntakeMotor;
    private final MotionMagicVoltage mmRequest;

    private double targetDeg = 0.0;


    private IntakeState state = IntakeState.UNKNOWN;
    private IntakeState lastState = null;
    private final Timer stateTimer;
    String canBusName = Constants.Imu.canBusName;
    private TalonFXConfiguration config;

    public Intake() {
        updownIntakeMotor = new TalonFX(25, canBusName);
        rollerIntakeMotor = new TalonFX(26,canBusName); // this may be a neo? 
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        mmRequest = new MotionMagicVoltage(0.0);

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = 53.33;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        config.Slot0.kP = 50.0; //23.0
        config.Slot0.kI = 0.0; //2.0
        config.Slot0.kD = 0.0; //0.5
        config.Slot0.kG = 0.0;

        config.MotionMagic.MotionMagicCruiseVelocity = 5.0; // rot /s
        config.MotionMagic.MotionMagicAcceleration = 3.0; // rot/s^2

        updownIntakeMotor.getConfigurator().apply(config);
        updownIntakeMotor.setNeutralMode(NeutralModeValue.Brake);

        updownIntakeMotor.getConfigurator().setPosition(0.0);

        stateTimer = new Timer();

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1/3; 
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;   

        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0; // might stop it after going down, not sure what to do here?
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true; 
        
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
        }

        switch (state) {
            case UNKNOWN -> { //insta go home
                setState(IntakeState.HOMING);
                enableSoftLimit(true);
            }
            case HOMING -> { 
                enableSoftLimit(true);
                zeroHere();
                setState(IntakeState.HOME);
            }
// combine these 2 later? (Tushar: probably not i feel like we should keep it where it is)
            case HOME -> {
                enableSoftLimit(true);
                setAngleDegrees(0);
                setState(IntakeState.IDLE);
            }
            case IDLE -> {
                enableSoftLimit(true);
                setAngleDegrees(1);
                 // Not 0 because it should home at 0 when it's set as far back as possible, this gives some tolerance if pid overshoots. Same w/ kick
            }
            case INTAKING -> {
                enableSoftLimit(true);
                setAngleDegrees(120);
                
            }
            case KICK -> {
                enableSoftLimit(true);
                setAngleDegrees(1);
                
                if (atTargetDegrees(1) && stateTimer.hasElapsed(0.5)) {
                    setState(IntakeState.INTAKING);
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
        setState(IntakeState.IDLE);
    }
    public void requestAKick() {
        setState(IntakeState.KICK);
    }
    public void enableSoftLimit(boolean enable){
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1/3; 
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = enable;   

        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0; // might stop it after going down, not sure what to do here?
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = enable; 
    }

    @Override
    public void periodic() {
        updateStateMachine();
    }
}
