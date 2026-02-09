package frc.robot.subsystems.ballControl;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase;

import frc.utils.StormSubsystem;

public class Intake extends StormSubsystem {

    private final TalonFX updownIntakeMotor;
    // private final TalonFX rollerIntakeMotor;
    private IntakeState currentState; 
    private final MotionMagicVoltage mmRequest;

    private double targetDeg = 0.0;
    private boolean hasBeenHomed = false;
    private final robotState

    public Intake() {
        updownIntakeMotor = new TalonFX(3);
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        mmRequest = new MotionMagicVoltage(0.0);
        robotState = RobotState.getInstance();
        // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Feedback.SensorToMechanismRatio = 40.0;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        config.Slot0.kP = 5.0; //23.0 
        config.Slot0.kI = 1.5; //2.0
        config.Slot0.kD = 0.3; //0.5
        config.Slot0.kG = 0.0;

        config.MotionMagic.MotionMagicCruiseVelocity = 0.1; // rot /s values should be much higher for the actual intake
        config.MotionMagic.MotionMagicAcceleration = 0.1; // rot/s^2

        updownIntakeMotor.getConfigurator().apply(config);
        updownIntakeMotor.setNeutralMode(NeutralModeValue.Brake);

        updownIntakeMotor.getConfigurator().setPosition(0.0);
    }

    public void setAngleDegrees(double degrees) {
        targetDeg = degrees;

        double accRotations = degrees / 360.0;

        updownIntakeMotor.setControl(mmRequest.withPosition(accRotations));
    }

    public void moveDeltaDegrees(double deltaDeg) {
        setAngleDegrees(getAngleDegrees() + deltaDeg);
    }

    public void zeroHere() {
        updownIntakeMotor.getConfigurator().setPosition(0.0);
        targetDeg = 0.0;
    }

    public double getAngleDegrees() {
        return updownIntakeMotor.getPosition().getValue().in(Degrees);
    }
    public enum IntakeLevel {
        UNKNOWN(-1.0),
        FLOOR(Double.NEGATIVE_INFINITY),
        
        HOME(Constants.Intake.home),
        
        OUT1(Constants.Intake.level1),
        
        OUT2(Constants.Intake.level2),
        

        private double position;

        IntakeLevel(double position) {
            this.position = position;
        }

        public double getValue() {
            return position;
        }
    }
        public enum IntakeState {
        UNKNOWN,
        HOMING,
        HOME,
        IDLE,
        KICK,
        HOLD,
    }
    public void enableSoftLimits(boolean enable) {

            config.ConfigForwardSoftLimitThreshold = IntakeLevel.HOME.getValue();
            config.ForwardSoftLimitEnable = enable;
            config.ConfigReverseSoftLimitThreshold = IntakeLevel.OUT2.getValue();
            config.ReverseSoftLimitEnable = enable;

        updownIntakeMotor.getConfigurator().apply(config);
    }
    public void stopIntake() {
        
        // This should be redundant, but for safety stop ASAP
        updownIntakeMotor.set(0);
    }   
 public void setState(IntakeState state) {
        this.currentState = state;
        robotState.setIntakeState(state);

        switch (state) {
            case UNKNOWN:
                console("***** UNKNOWN state *****");
                hasBeenHomed = false;
                robotState.setIntakeHasBeenHomed(hasBeenHomed);
                enableSoftLimits(true);
                targetPosition = IntakeLevel.UNKNOWN.position;
                break;

            case HOMING:
                console("***** HOMING state *****");
                enableSoftLimits(false);
                break;

            case HOME:
                console("***** HOME state *****");
                hasBeenHomed = true;
                robotState.setIntakeHasBeenHomed(hasBeenHomed);
                mmRequest.setPosition(IntakeLevel.HOME.getValue());
                enableSoftLimits(true);
                break;

            case KICK:
                console("***** KICK state *****");
                hasBeenHomed = true;
                robotState.setIntakeHasBeenHomed(hasBeenHomed);
                mmRequest.setPosition(IntakeLevel.HOME.getValue());
                enableSoftLimits(true);
                break;
        }
    }

}  


