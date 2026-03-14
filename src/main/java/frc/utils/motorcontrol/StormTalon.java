package frc.utils.motorcontrol;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import org.littletonrobotics.junction.Logger;

public class StormTalon extends TalonFX implements MotorLogger<StormTalon> {
    private String name;

    // Cached status signals — call refreshSignals() once per cycle
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<Voltage> motorVoltageSignal;

    public StormTalon(int deviceId) {
        this(deviceId, new CANBus(""));
    }

    public StormTalon(int deviceId, CANBus canBus) {
        super(deviceId, canBus);
        velocitySignal = getVelocity();
        supplyCurrentSignal = getSupplyCurrent();
        motorVoltageSignal = getMotorVoltage();
    }

    @Override
    public String getMotorName() {
        return name;
    }

    public StormTalon withName(String name) {
        this.name = name;
        return this;
    }

    /** Refresh all cached signals in a single CAN batch read. */
    public void refreshSignals() {
        BaseStatusSignal.refreshAll(velocitySignal, supplyCurrentSignal, motorVoltageSignal);
    }

    @Override
    public double getMotorCurrent() {
        return supplyCurrentSignal.getValueAsDouble();
    }

    @Override
    public double getMotorPosition() {
        return getPosition().getValueAsDouble();
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
        Logger.recordOutput(basePath + "/Current", getMotorCurrent());
        Logger.recordOutput(basePath + "/Position", getMotorPosition());
        Logger.recordOutput(basePath + "/Velocity", getMotorVelocity());
        Logger.recordOutput(basePath + "/Voltage", getMotorVoltageOutput());
        Logger.recordOutput(basePath + "/ClosedLoopReference", getClosedLoopReference().getValueAsDouble());
        Logger.recordOutput(basePath + "/ClosedLoopError", getClosedLoopError().getValueAsDouble());
        Logger.recordOutput(basePath + "/ClosedLoopOutput", getClosedLoopOutput().getValueAsDouble());
        Logger.recordOutput(basePath + "/ClosedLoopReferenceSlope", getClosedLoopReferenceSlope().getValueAsDouble());
    }
}
