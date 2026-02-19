package frc.utils.motorcontrol;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;

public class StormSparkMax extends SparkMax implements MotorLogger {
    private String name;

    public StormSparkMax(int deviceId, MotorType motorType) {
        super(deviceId, motorType);
    }

    @Override
    public String getMotorName() {
        return name;
    }

    public StormSparkMax withName(String name) {
        this.name = name;
        return this;
    }

    @Override
    public double getMotorCurrent() {
        return getOutputCurrent();
    }

    @Override
    public double getMotorPosition() {
        return getEncoder().getPosition();
    }

    @Override
    public double getMotorVoltageOutput() {
        return getAppliedOutput() * getBusVoltage();
    }
}
