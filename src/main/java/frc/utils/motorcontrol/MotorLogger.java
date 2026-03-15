package frc.utils.motorcontrol;

import org.littletonrobotics.junction.Logger;

public interface MotorLogger<T extends MotorLogger<T>> {
    String getMotorName();
    T withName(String name);
    double getMotorCurrent();
    double getMotorPosition();
    double getMotorVelocity();
    double getMotorVoltageOutput();

    default void periodic() {
        String basePath = "Motors/" + getMotorName();
        Logger.recordOutput(basePath + "/StatorCurrent", getMotorCurrent());
        Logger.recordOutput(basePath + "/Position", getMotorPosition());
        Logger.recordOutput(basePath + "/Velocity", getMotorVelocity());
        Logger.recordOutput(basePath + "/Voltage", getMotorVoltageOutput());
    }
}
