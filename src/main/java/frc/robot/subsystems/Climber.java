package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.utils.StormSubsystem;

public class Climber extends StormSubsystem {

    private final TalonFX leadMotor;
    private final TalonFX followerMotor;

    public enum ClimberState {
        UNKNOWN,
        IDLE,
        HOMING,
        READYTOCLIMB,
        CLIMBING,
        CLIMBED,
        MANUALCLIMB,
    }

    public enum Direction {
        FORWARD,
        BACKWARD,
    }

    private final TalonFXConfiguration config = new TalonFXConfiguration();
    private final PositionTorqueCurrentFOC posRequest = new PositionTorqueCurrentFOC(0);

    public ClimberState state = ClimberState.IDLE;
    public boolean hasBeenHomed = true;
    private double manualVoltage = 0.0;

    public Climber() {
        leadMotor = new TalonFX(Constants.Climber.leftMotorID, new CANBus(""));
        followerMotor = new TalonFX(Constants.Climber.rightMotorID, new CANBus(""));
        followerMotor.setControl(new Follower(leadMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        config.Feedback.SensorToMechanismRatio = 16;
        config.Slot0.kP = Constants.Climber.kP;
        leadMotor.getConfigurator().apply(config);
        NeutralModeValue neutralMode = Constants.Climber.coastMode ? NeutralModeValue.Coast : NeutralModeValue.Brake;
        leadMotor.setNeutralMode(neutralMode);
        followerMotor.setNeutralMode(neutralMode);

        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit = Constants.Climber.statorCurrentLimit;
        currentLimits.StatorCurrentLimitEnable = true;
        leadMotor.getConfigurator().apply(currentLimits);
        followerMotor.getConfigurator().apply(currentLimits);
    }

    public void setState(ClimberState newState) {
        state = newState;
    }

    public void setPositionDegrees(double degrees) {
        leadMotor.setControl(posRequest.withPosition(degrees / 360.0));
    }

    public void zeroHere() {
        leadMotor.getConfigurator().setPosition(0.0);
    }

    public double getAngleDegrees() {
        return leadMotor.getPosition().getValueAsDouble() * 360.0;
    }

    public boolean atTargetDegrees(double deg) {
        return Math.abs(getAngleDegrees() - deg) < 2.0;
    }

    public void setManualDirection(Direction dir) {
        if (dir == null) {
            setState(ClimberState.IDLE);
        } else if (dir == Direction.FORWARD) {
            manualVoltage = 3.0;
            setState(ClimberState.MANUALCLIMB);
        } else {
            manualVoltage = -3.0;
            setState(ClimberState.MANUALCLIMB);
        }
    }

    public boolean isAtHome() {
        return leadMotor.getStatorCurrent().getValueAsDouble() > 15.0;
    }

    @Override
    public void periodic() {
        super.periodic();
        switch (state) {
            case UNKNOWN -> leadMotor.setVoltage(0);
            case HOMING -> leadMotor.setVoltage(-2.0);
            case IDLE -> setPositionDegrees(Constants.Climber.homedDegrees);
            case READYTOCLIMB -> setPositionDegrees(Constants.Climber.readyToClimbDegrees);
            case CLIMBING -> setPositionDegrees(Constants.Climber.climbedDegrees);
            case CLIMBED -> setPositionDegrees(Constants.Climber.climbedDegrees);
            case MANUALCLIMB -> leadMotor.setVoltage(manualVoltage);
        }
    }
}
