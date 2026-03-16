package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Tracks who won autonomous and computes the current shift schedule.
 * Adapted from 6328 RobotCode2026Public HubShiftUtil.
 *
 * Priority for "who won auto": override → FMS game data → default (false).
 * FMS message first char: 'B' = Blue won auto, 'R' = Red won auto.
 *
 * Shift schedule (teleop = 140s):
 *   SHIFT1  0–10s   both active
 *   SHIFT2  10–35s  winner active, loser inactive
 *   SHIFT3  35–60s  loser active, winner inactive
 *   SHIFT4  60–85s  winner active, loser inactive
 *   SHIFT5  85–110s loser active, winner inactive
 *   ENDGAME 110–140s both active
 */
public class GameDataTracker {

    public enum Shift {
        SHIFT1, SHIFT2, SHIFT3, SHIFT4, SHIFT5, ENDGAME, AUTO, DISABLED
    }

    public record ShiftInfo(Shift currentShift, double elapsedTime, double remainingTime, boolean ourShift) {}

    // --- Override supplier ---
    private static Supplier<Optional<Boolean>> allianceWinOverride = Optional::empty;

    // --- Shift timing ---
    private static final Timer shiftTimer = new Timer();
    private static double shiftTimerOffset = 0.0;

    private static final double[] shiftStartTimes = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0};
    private static final double[] shiftEndTimes   = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};

    // Schedule when our alliance won auto (acts first after SHIFT1)
    private static final boolean[] wonSchedule  = {true, false, true, false, true, true};
    // Schedule when our alliance lost auto
    private static final boolean[] lostSchedule = {true, true, false, true, false, true};

    private static final double timeResetThreshold = 3.0;
    private static final double autoEndTime = 20.0;

    private GameDataTracker() {}

    // ---- Override / FMS data ----

    public static void setAllianceWinOverride(Supplier<Optional<Boolean>> supplier) {
        allianceWinOverride = supplier;
    }

    public static Optional<Boolean> getAllianceWinOverride() {
        return allianceWinOverride.get();
    }

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

    // ---- Shift timer ----

    /** Call at the start of teleop to begin shift tracking. */
    public static void initialize() {
        shiftTimerOffset = 0.0;
        shiftTimer.restart();
    }

    private static boolean[] getSchedule() {
        return ourAllianceWonAuto() ? wonSchedule : lostSchedule;
    }

    /** Returns the current shift state with timing info. */
    public static ShiftInfo getShiftInfo() {
        if (DriverStation.isAutonomousEnabled()) {
            double currentTime = shiftTimer.get() - shiftTimerOffset;
            return new ShiftInfo(Shift.AUTO, currentTime, autoEndTime - currentTime, true);
        }

        if (!DriverStation.isTeleopEnabled()) {
            return new ShiftInfo(Shift.DISABLED, 0, 0, false);
        }

        double timerValue = shiftTimer.get();
        double currentTime = timerValue - shiftTimerOffset;

        // Sync with FMS clock if drifted
        double fmsTeleopTime = 140.0 - DriverStation.getMatchTime();
        if (Math.abs(fmsTeleopTime - currentTime) >= timeResetThreshold
                && fmsTeleopTime <= 135.0
                && DriverStation.isFMSAttached()) {
            shiftTimerOffset += currentTime - fmsTeleopTime;
            currentTime = timerValue - shiftTimerOffset;
        }

        boolean[] schedule = getSchedule();

        // Find current shift index
        int idx = shiftStartTimes.length - 1; // default to last (endgame)
        for (int i = 0; i < shiftStartTimes.length; i++) {
            if (currentTime >= shiftStartTimes[i] && currentTime < shiftEndTimes[i]) {
                idx = i;
                break;
            }
        }

        double elapsed = currentTime - shiftStartTimes[idx];
        double remaining = shiftEndTimes[idx] - currentTime;

        // Combine with adjacent same-state shifts
        if (idx > 0 && schedule[idx] == schedule[idx - 1]) {
            elapsed = currentTime - shiftStartTimes[idx - 1];
        }
        if (idx < shiftEndTimes.length - 1 && schedule[idx] == schedule[idx + 1]) {
            remaining = shiftEndTimes[idx + 1] - currentTime;
        }

        Shift currentShift = Shift.values()[idx];
        return new ShiftInfo(currentShift, elapsed, remaining, schedule[idx]);
    }
}
