package frc.utils.motorcontrol;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

public class StormTalon extends TalonFX implements MotorLogger<StormTalon> {
    private String name;

    public StormTalon(int deviceId) {
        super(deviceId);
    }

    public StormTalon(int deviceId, CANBus canBus) {
        super(deviceId, canBus);
    }

    @Override
    public String getMotorName() {
        return name;
    }

    public StormTalon withName(String name) {
        this.name = name;
        return this;
    }

    @Override
    public double getMotorCurrent() {
        return getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public double getMotorPosition() {
        return getPosition().getValueAsDouble();
    }

    @Override
    public double getMotorVoltageOutput() {
        return getMotorVoltage().getValueAsDouble();
    }
}
