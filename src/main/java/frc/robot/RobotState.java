package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
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
    public enum HopperState{
        IDLE,
        HOPPER_FULL,
        HOPPER_HALF,
        HOPPER_EMPTY,
        UNKNOWN
    }

    private static RobotState instance;
    private final StateSimMode stateSimMode;
    private StatePeriod statePeriod;
    private BatteryState batteryState;
    public HopperState hopperState;

    private StateAlliance alliance = StateAlliance.MISSING;

    private boolean joystickReady = false;
    private Pose2d currentPose;

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

    public boolean isAllianceMissing() {
        return alliance == StateAlliance.MISSING;
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

    public boolean getJoystickReady() {
        return joystickReady;
    }

    public void setJoystickReady(boolean ready) {
        joystickReady = ready;
    }

    // Make a copy, not a reference to the same object!
    public void setPose(Pose2d pose) {
        currentPose = new Pose2d(pose.getX(), pose.getY(), new Rotation2d(pose.getRotation().getRadians()));
    }

    // Make a copy, not a reference to the same object!
    public Pose2d getPose() {
        return new Pose2d(currentPose.getX(), curretPose.getY(), new Rotation2d(currePose.getRotation().getRadians()));
    }
  
    public void setHopperState(HopperState hopperState){
        this.hopperState = hopperState;
    }
  
    public HopperState getHopperState(){
        return hopperState;
    }

    public double getDistFromHub() {
        Pose2d hubPose = (alliance == StateAlliance.BLUE) ? FieldConstants.hubPositionBLUE() : FieldConstants.hubPositionRED();
        return currentPose.minus(hubPose).getTranslation().getNorm();
    }

}
