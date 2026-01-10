package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public class RobotState {

    private static RobotState instance;
    private final StateSimMode stateSimMode;
    private StatePeriod statePeriod;
    private boolean upperSensorTriggered;

    public enum StateSimMode {
        REAL,
        SIMULATION,
        AKIT_REPLAY,
        AKIT_SIM
    }

    public enum StatePeriod {
        DISABLED,
        AUTONOMOUS,
        TELEOP,
        TEST
    }

    private RobotState() {
        if (RobotBase.isReal()) {
            stateSimMode = StateSimMode.REAL;
        } else if (Constants.Toggles.useAdvantageKit) {
            if (Constants.Akit.doReplay) {
                stateSimMode = StateSimMode.AKIT_REPLAY;
            } else {
                stateSimMode = StateSimMode.AKIT_SIM;
            }
        } else { // basic simulation
            stateSimMode = StateSimMode.SIMULATION;
        }
    }

    public static RobotState getInstance() {
        if (instance != null) {
            return instance;
        }

        instance = new RobotState();
        return instance;
    }

    public StateSimMode getSimMode() {
        return stateSimMode;
    }

    public StatePeriod getPeriod() {
        return statePeriod;
    }

    public void setPeriod(StatePeriod period) {
        statePeriod = period;
    }
}
