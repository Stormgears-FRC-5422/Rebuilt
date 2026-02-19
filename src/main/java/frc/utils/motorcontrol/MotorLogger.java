package frc.utils.motorcontrol;

import org.littletonrobotics.junction.Logger;

public interface MotorLogger {
    String getMotorName();
    double getMotorCurrent();
    double getMotorPosition();
    double getMotorVoltageOutput();

    default void logMotorData() {
        String basePath = "Motors/" + getMotorName();
        Logger.recordOutput(basePath + "/Current", getMotorCurrent());
        Logger.recordOutput(basePath + "/Position", getMotorPosition());
        Logger.recordOutput(basePath + "/Voltage", getMotorVoltageOutput());
    }
}
