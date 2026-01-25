// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.misc;

import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotState.StatePeriod;
import frc.utils.StormSubsystem;
import org.littletonrobotics.conduit.ConduitApi;

public class BatteryMonitor extends StormSubsystem {
    RobotState state;
    ConduitApi conduit;
    StatePeriod lastPeriod;
    double voltage;
    BatteryState batteryState;

    public BatteryMonitor() {
        state = RobotState.getInstance();
        conduit = ConduitApi.getInstance();
        lastPeriod = state.getPeriod();
    }

    @Override
    public void periodic() {
        super.periodic();

        StatePeriod newPeriod = state.getPeriod();

        switch (newPeriod) {
            case AUTONOMOUS:
            case TELEOP:
            case TEST:
                // Prevent going into enabled mode if we had a low voltage the last time through
                // Only in debug mode, so we just keep going during competition
                if (batteryState == BatteryState.BAD
                    && newPeriod != lastPeriod
                    && Constants.Debug.debug
                    && Constants.Power.autoKill) {
                    throw new BadBatteryException("The battery must be changed: " + voltage + " Volts");
                }
            default:
                console("battery level: " + state.getBatteryState() + ", " + voltage + " Volts", 1000);
        }

        voltage = conduit.getPDPVoltage();

        if (voltage > Constants.Power.warnLimit) {
            batteryState = BatteryState.GOOD;
        } else if (voltage > Constants.Power.shutoffLimit) {
            batteryState = BatteryState.WARN;
        } else {
            batteryState = BatteryState.BAD;
        }

        state.setBatteryState(batteryState);
        lastPeriod = newPeriod;
    }

    public enum BatteryState {
        GOOD, WARN, BAD
    }

    public static class BadBatteryException extends RuntimeException {
        public BadBatteryException(String message) {
            super(message);
        }
    }
}

