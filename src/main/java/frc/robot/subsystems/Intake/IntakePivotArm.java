package frc.robot.subsystems.Intake;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.SparkConstants;
import frc.robot.RobotState;
import frc.utils.StormSubsystem;
import org.littletonrobotics.junction.Logger;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


public class IntakePivotArm extends StormSubsystem{
    private final SparkMax intakeMax;
    private final RobotState robotState;
    private final ElevatorFeedforward feedForward;
    private double IntakeSpeed;
    private boolean hasBeenHomed = false;
    private double currentPosition;
    private double targetPosition;
    private double currentVelocity;
    private double actualVelocity;
    private double lastTimestamp = 0;
    private double lastPosition;
    private final RelativeEncoder intakeRelativeEncoder;
    private SparkClosedLoopController controller;
    private SparkMaxConfig intakeConfig;
    private IntakeState currentState; 
    
    public IntakePivotArm(){
        robotState = RobotState.getInstance();
        intakeMax = new SparkMax(Constants.Intake.MotorID, SparkLowLevel.MotorType.kBrushless);
        intakeRelativeEncoder = intakeMax.getEncoder();
        intakeConfig = new SparkMaxConfig();
        SparkMaxConfig globalConfig = new SparkMaxConfig();
        globalConfig.smartCurrentLimit(SparkConstants.CurrentLimit)
            .voltageCompensation(SparkConstants.NominalVoltage)
            .idleMode(Constants.Intake.brakeMode ? IdleMode.kBrake : IdleMode.kCoast);
        intakeConfig.apply(globalConfig)
            .inverted(Constants.Intake.invertLeader);
        intakeConfig.limitSwitch
            .forwardLimitSwitchType(Type.kNormallyOpen)
            .forwardLimitSwitchEnabled(false)
            .reverseLimitSwitchType(Type.kNormallyOpen)
            .reverseLimitSwitchEnabled(false);

        // Soft limits
        intakeConfig.softLimit
            .forwardSoftLimit(IntakeLevel.OUT3.getValue())
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(IntakeLevel.HOME.getValue())
            .reverseSoftLimitEnabled(true);
        double kP = Constants.Intake.kP ; // volts/rot 0.5 - 1
        double kI = Constants.Intake.kI;  // volts/ rot*s  0.01 - 0.1
        double kD = Constants.Intake.kD;  // volts / rot/s  0.001 - 0.01
        double maxV = Constants.Intake.maxV;  // volts
        double maxVPct = maxV / SparkConstants.NominalVoltage; // percentage (-1 to 1)
        intakeConfig.closedLoop
            .p(kP)
            .i(kI)
            .d(kD)
            .outputRange(-maxVPct, maxVPct);
        intakeConfig.closedLoopRampRate(Constants.Intake.closedLoopRampRate);

        intakeMax.configure(intakeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        double kS = Constants.Intake.kS;  // volts
        double kG = Constants.Intake.kG;  // volts
        double kV = Constants.Intake.kV;  // volts / rot/s
        double kA = Constants.Intake.kA;  // negligible - ignore - 0.032 volts / rot/s
        feedForward = new ElevatorFeedforward(kS, kG, kV, kA);

        controller = intakeMax.getClosedLoopController();
        setState(IntakeState.UNKNOWN);
    }
    @Override
    public void periodic() {
        super.periodic();
        double now = Timer.getFPGATimestamp();
        currentPosition = intakeRelativeEncoder.getPosition();
        currentVelocity = intakeRelativeEncoder.getVelocity();

        actualVelocity = 60 * (currentPosition - lastPosition) / (now - lastTimestamp);

        double ffVoltage = 0;

        switch (currentState) {
            case HOMING:
                home();
                intakeMax.set(IntakeSpeed);
                break;

            case SIMPLE_MOTION:
                if (hasBeenHomed) {
                    simpleMove(currentPosition);
                    intakeMax.set(IntakeSpeed);
                } else {
                    console("Stubbornly refusing to move before homed!", 25);
                }
                break;

            case PID_MOTION:
                if (hasBeenHomed) {
                    ffVoltage = feedForward.calculate(0); // 0 here basically gives us gravity compensation
                    controller.setReference(targetPosition,
                        SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, ffVoltage);
                } else {
                    console("Stubbornly refusing to move before homed!", 25);
                }
                break;

            default:
                intakeMax.set(0);
        }

        Logger.recordOutput("Intake/TargetPosition", targetPosition);
        Logger.recordOutput("Intake/LeaderEncoder", currentPosition);
        Logger.recordOutput("Intake/LeaderVelocity", currentVelocity);
        Logger.recordOutput("Intake/ActualVelocity", actualVelocity);
        Logger.recordOutput("Intake/Feedforward", ffVoltage);

        lastTimestamp = now;
        lastPosition = currentPosition;
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
                intakeRelativeEncoder.setPosition(IntakeLevel.HOME.getValue());
                enableSoftLimits(true);
                break;

            case SIMPLE_MOTION:
                console("***** SIMPLE_MOTION state *****");
                break;

            case PID_MOTION:
                console("***** PID_MOTION state *****");
                break;

            case IDLE:
                console("***** IDLE state *****");
                targetPosition = IntakeLevel.UNKNOWN.position;
                stopIntake();
                break;
        }
    }

    public boolean isAtHome() {
        if (Constants.Intake.useCurrentLimitHomeStrategy) {
            // might want to use an average here to minimize spikes
            console("isAtHome output current: " + intakeMax.getOutputCurrent());
            return intakeMax.getOutputCurrent() > Constants.Intake.stallCurrentLimit;
        } else {
            console("isAtHome current position: " + currentPosition);
            return Math.abs(currentPosition) < Constants.Intake.homePositionThreshold;
        }
    }

    private void home() {
        if (Constants.Intake.useCurrentLimitHomeStrategy) {
            // might want to use an average here to minimize spikes
            setSpeed(Constants.Intake.stallCurrentSpeed);
        } else {
            setSpeed(0.0);
        }
    }

    public boolean isAtTarget() {
        // It isn't enought to be at the right location. We need to be stopped, too.
        return ( Math.abs(currentPosition - targetPosition) < Constants.Intake.allowedError ) &&
               ( Math.abs(actualVelocity) < Constants.Intake.velocityThreshold);
    }


    private void simpleMove(double currentPosition) {
        if (currentPosition < targetPosition) {
            setSpeed(Constants.Intake.simpleUpSpeed);
        } else if (currentPosition > targetPosition) {
            setSpeed(-Constants.Intake.simpleDownSpeed);
        } else {
            setSpeed(0);
        }
    }

    private void setSpeed(double speed) {
        this.IntakeSpeed = speed;
    }

    public void setTargetLevel(IntakeLevel targetLevel) {
        setTargetPosition(targetLevel.getValue());
    }

    public void setTargetPosition(double position) {
        this.targetPosition = position;
    }

    public double getCurrentPosition() {return currentPosition;}

    public void stopIntake() {
        setSpeed(0);
        // This should be redundant, but for safety stop ASAP
        intakeMax.set(0);
    }

    public void enableSoftLimits(boolean enable) {
        intakeConfig.softLimit
            .forwardSoftLimit(IntakeLevel.HOME.getValue())
            .forwardSoftLimitEnabled(enable)
            .reverseSoftLimit(IntakeLevel.OUT3.getValue())
            .reverseSoftLimitEnabled(enable);

        intakeMax.configure(intakeConfig, SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters);
    }

    public enum IntakeLevel {
        UNKNOWN(-1.0),
        FLOOR(Double.NEGATIVE_INFINITY),
        
        HOME(Constants.Intake.home),
        
        OUT1(Constants.Intake.level1),
        
        OUT2(Constants.Intake.level2),
        
        OUT3(Constants.Intake.level3);
        

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
        SIMPLE_MOTION,
        PID_MOTION,
        HOLD,
    }
}

        


