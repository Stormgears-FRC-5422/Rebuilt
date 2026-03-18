package frc.utils.motorcontrol;

import org.littletonrobotics.junction.Logger;

public interface MotorLogger<T extends MotorLogger<T>> {
    String getMotorName();
    T withName(String name);
    double getMotorCurrent();
    double getMotorPosition();
    double getMotorVelocity();
    double getMotorVoltageOutput();

    /** Scale factor applied to position/velocity in log output only. Does not affect getMotorPosition()/getMotorVelocity(). */
    default double getLoggingScale() { return 1.0; }

    default void periodic() {
        double scale = getLoggingScale();
        String basePath = "Motors/" + getMotorName();
        Logger.recordOutput(basePath + "/StatorCurrent", getMotorCurrent());
        Logger.recordOutput(basePath + "/Position", getMotorPosition() * scale);
        Logger.recordOutput(basePath + "/Velocity", getMotorVelocity() * scale);
        Logger.recordOutput(basePath + "/Voltage", getMotorVoltageOutput());
    }
}
