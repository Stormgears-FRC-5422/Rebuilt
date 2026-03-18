package frc.utils.motorcontrol;

import com.revrobotics.spark.SparkMax;

public class StormSparkMax extends SparkMax implements MotorLogger<StormSparkMax> {
    private String name;
    private double loggingScale = 1.0;

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

    /** Scale factor applied to position/velocity in log output only. Does not affect getMotorPosition()/getMotorVelocity(). */
    public StormSparkMax withLoggingScale(double scale) {
        this.loggingScale = scale;
        return this;
    }

    @Override
    public double getLoggingScale() {
        return loggingScale;
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
    public double getMotorVelocity() {
        return getEncoder().getVelocity();
    }

    @Override
    public double getMotorVoltageOutput() {
        return getAppliedOutput() * getBusVoltage();
    }
}
