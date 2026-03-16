package frc.utils.motorcontrol;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class StormTalon extends TalonFX implements MotorLogger<StormTalon> {
    private String name;
    private boolean logClosedLoop = false;

    // Cached status signals — refreshed in a single CAN batch read each cycle
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Current> statorCurrentSignal;
    private final StatusSignal<Voltage> motorVoltageSignal;
    private final StatusSignal<Angle> positionSignal;

    // Cached closed-loop signals — only refreshed/logged when withClosedLoopLogging() is set
    private final StatusSignal<Double> closedLoopReferenceSignal;
    private final StatusSignal<Double> closedLoopErrorSignal;
    private final StatusSignal<Double> closedLoopOutputSignal;
    private final StatusSignal<Double> closedLoopReferenceSlopeSignal;

    public StormTalon(int deviceId) {
        this(deviceId, new CANBus(""));
    }

    public StormTalon(int deviceId, CANBus canBus) {
        super(deviceId, canBus);
        velocitySignal = getVelocity();
        statorCurrentSignal = getStatorCurrent();
        motorVoltageSignal = getMotorVoltage();
        positionSignal = getPosition();
        closedLoopReferenceSignal = getClosedLoopReference();
        closedLoopErrorSignal = getClosedLoopError();
        closedLoopOutputSignal = getClosedLoopOutput();
        closedLoopReferenceSlopeSignal = getClosedLoopReferenceSlope();
    }

    @Override
    public String getMotorName() {
        return name;
    }

    public StormTalon withName(String name) {
        this.name = name;
        return this;
    }

    /** Enable closed-loop signal logging for this motor (refreshed and logged each cycle). */
    public StormTalon withClosedLoopLogging() {
        this.logClosedLoop = true;
        double hz = Constants.Motor.closedLoopUpdateHz;
        closedLoopReferenceSignal.setUpdateFrequency(hz);
        closedLoopErrorSignal.setUpdateFrequency(hz);
        closedLoopOutputSignal.setUpdateFrequency(hz);
        closedLoopReferenceSlopeSignal.setUpdateFrequency(hz);
        return this;
    }

    /** Refresh all cached signals in a single CAN batch read. */
    public void refreshSignals() {
        if (logClosedLoop) {
            BaseStatusSignal.refreshAll(
                velocitySignal, statorCurrentSignal, motorVoltageSignal, positionSignal,
                closedLoopReferenceSignal, closedLoopErrorSignal,
                closedLoopOutputSignal, closedLoopReferenceSlopeSignal
            );
        } else {
            BaseStatusSignal.refreshAll(velocitySignal, statorCurrentSignal, motorVoltageSignal, positionSignal);
        }
    }

    @Override
    public double getMotorCurrent() {
        return statorCurrentSignal.getValueAsDouble();
    }

    @Override
    public double getMotorPosition() {
        return positionSignal.getValueAsDouble();
    }

    @Override
    public double getMotorVelocity() {
        return velocitySignal.getValueAsDouble();
    }

    @Override
    public double getMotorVoltageOutput() {
        return motorVoltageSignal.getValueAsDouble();
    }

    @Override
    public void periodic() {
        refreshSignals();
        String basePath = "Motors/" + getMotorName();
        Logger.recordOutput(basePath + "/StatorCurrent", getMotorCurrent());
        Logger.recordOutput(basePath + "/Position", getMotorPosition());
        Logger.recordOutput(basePath + "/Velocity", getMotorVelocity());
        Logger.recordOutput(basePath + "/Voltage", getMotorVoltageOutput());
        if (logClosedLoop) {
            Logger.recordOutput(basePath + "/ClosedLoopReference", closedLoopReferenceSignal.getValueAsDouble());
            Logger.recordOutput(basePath + "/ClosedLoopError", closedLoopErrorSignal.getValueAsDouble());
            Logger.recordOutput(basePath + "/ClosedLoopOutput", closedLoopOutputSignal.getValueAsDouble());
            Logger.recordOutput(basePath + "/ClosedLoopReferenceSlope", closedLoopReferenceSlopeSignal.getValueAsDouble());
        }
    }
}
