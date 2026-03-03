package frc.robot.elastic;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.GameDataTracker;
import frc.utils.pid.DashboardTuner;

/**
 * Publishes game data state to Elastic dashboard under "Elastic/Co-Pilot Dashboard/".
 * Tiles: Match Time (number), Data Received (bool), We Won Auto (bool),
 *        Override Active (bool), Auto Winner (text).
 */
public class GameDataDashboard extends DashboardTuner {

    private final DoublePublisher matchTimePub;
    private final BooleanPublisher dataReceivedPub;
    private final BooleanPublisher weWonAutoPub;
    private final BooleanPublisher overrideActivePub;
    private final StringPublisher autoWinnerPub;

    public GameDataDashboard() {
        super("Co-Pilot Dashboard");
        matchTimePub      = table.getDoubleTopic("Match Time").publish();
        dataReceivedPub   = table.getBooleanTopic("Data Received").publish();
        weWonAutoPub      = table.getBooleanTopic("We Won Auto").publish();
        overrideActivePub = table.getBooleanTopic("Override Active").publish();
        autoWinnerPub     = table.getStringTopic("Auto Winner").publish();
    }

    public void periodic() {
        boolean received = GameDataTracker.isGameDataReceived();
        boolean won      = GameDataTracker.ourAllianceWonAuto();
        boolean override = GameDataTracker.getAllianceWinOverride().isPresent();

        matchTimePub.set(DriverStation.getMatchTime());
        dataReceivedPub.set(received);
        weWonAutoPub.set(won);
        overrideActivePub.set(override);
        autoWinnerPub.set(
            received ? (won ? "WE WON AUTO" : "WE LOST AUTO")
            : override ? (won ? "OVERRIDE: WON" : "OVERRIDE: LOST")
            : "NO DATA");
    }
}
