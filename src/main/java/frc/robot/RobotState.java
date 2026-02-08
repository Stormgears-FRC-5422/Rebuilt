package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.Intake.IntakePivotArm;
import frc.robot.subsystems.Intake.IntakePivotArm.IntakeState;
import frc.robot.subsystems.misc.BatteryMonitor;
import frc.robot.subsystems.misc.BatteryMonitor.BatteryState;


public class RobotState {
    public enum StateAlliance {
        RED, BLUE, MISSING
    }

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

    private static RobotState instance;
    private Pose2d currentPose;

    private final StateSimMode stateSimMode;
    private StatePeriod statePeriod;
    private BatteryState batteryState;
    private boolean IntakeHasBeenHomed;
    private IntakeState intakeState = IntakeState.UNKNOWN;

    private StateAlliance alliance = StateAlliance.MISSING;

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

    public void setAlliance(StateAlliance alliance) {
        if (this.alliance != alliance) {
            System.out.println("**********");
            System.out.println("Alliance is now reported as " + alliance + ", was " + this.alliance);
            System.out.println("**********");
        }
        this.alliance = alliance;
    }

    public StateAlliance getAlliance() {
        return alliance;
    }

    public boolean isAllianceBlue() {
        return alliance == StateAlliance.BLUE;
    }

    public boolean isAllianceRed() {
        return alliance == StateAlliance.RED;
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

    public void setBatteryState(BatteryState batteryState) {
        this.batteryState = batteryState;
    }

    public BatteryState getBatteryState() {
        return this.batteryState;
    }

    public void setPose(Pose2d pose) {
        // Make a copy, not a reference to the same object!
        currentPose = new Pose2d(pose.getX(), pose.getY(), new Rotation2d(pose.getRotation().getRadians()));
    }

    public Pose2d getPose() {
        return currentPose;
    }
    public void setIntakeHasBeenHomed(boolean hasBeenHomed) {
        IntakeHasBeenHomed = hasBeenHomed;
    }

    public boolean IntakeHasBeenHomed() {
        return IntakeHasBeenHomed;
    }

    public IntakeState getIntakeState() {
        return intakeState;
    }
    public void setIntakeState(IntakeState state) {
        intakeState = state;
    }


}
