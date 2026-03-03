package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Tracks who won autonomous and provides FMS game data with operator override fallback.
 * Ported from 6328 RobotCode2026Public HubShiftUtil (without shift timing logic).
 *
 * Priority for "who won auto": override → FMS game data → default (false).
 * FMS message first char: 'B' = Blue won auto, 'R' = Red won auto.
 */
public class GameDataTracker {

    // Supplier returns Optional<Boolean>:
    //   Optional.of(true)  = our alliance won auto
    //   Optional.of(false) = our alliance lost auto
    //   Optional.empty()   = no override, rely on FMS
    private static Supplier<Optional<Boolean>> allianceWinOverride = Optional::empty;
    private static boolean ignoreHubState = false;

    private GameDataTracker() {}

    public static void setAllianceWinOverride(Supplier<Optional<Boolean>> supplier) {
        allianceWinOverride = supplier;
    }

    public static Optional<Boolean> getAllianceWinOverride() {
        return allianceWinOverride.get();
    }

    public static void setIgnoreHubState(boolean ignore) {
        ignoreHubState = ignore;
    }

    public static boolean getIgnoreHubState() {
        return ignoreHubState;
    }

    /** True if FMS sent a non-empty game-specific message this match. */
    public static boolean isGameDataReceived() {
        return DriverStation.getGameSpecificMessage().length() > 0;
    }

    /**
     * Did our alliance win auto?
     * Priority: operator override → FMS game data → default false.
     */
    public static boolean ourAllianceWonAuto() {
        Optional<Boolean> override = getAllianceWinOverride();
        if (override.isPresent()) return override.get();

        String msg = DriverStation.getGameSpecificMessage();
        if (msg.length() > 0) {
            char c = msg.charAt(0);
            DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
            if (c == 'B') return alliance == DriverStation.Alliance.Blue;
            if (c == 'R') return alliance == DriverStation.Alliance.Red;
        }
        return false;
    }
}
