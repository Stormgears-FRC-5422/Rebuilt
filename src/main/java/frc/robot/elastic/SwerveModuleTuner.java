package frc.robot.elastic;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import frc.utils.pid.PidConfig;
import frc.utils.pid.DashboardTuner;
import frc.utils.pid.PidTuningGroup;

public class SwerveModuleTuner extends DashboardTuner {
    private final PidTuningGroup drivePidTuning;
    private final PidTuningGroup steerPidTuning;
    private final BooleanPublisher driveApplyPub;
    private final BooleanSubscriber driveApplySub;
    private final BooleanPublisher steerApplyPub;
    private final BooleanSubscriber steerApplySub;
    private final PidConfig drivePidConfig;
    private final PidConfig steerPidConfig;

    public SwerveModuleTuner(String name, PidConfig drivePidConfig, PidConfig steerPidConfig) {
        super(name);
        this.drivePidConfig = drivePidConfig;
        this.steerPidConfig = steerPidConfig;
        this.drivePidTuning = new PidTuningGroup(this, "Drive PID", drivePidConfig);
        this.steerPidTuning = new PidTuningGroup(this, "Steer PID", steerPidConfig);

        // Create apply buttons at root level of the tuner
        var driveApplyTopic = table.getBooleanTopic("Drive Apply");
        this.driveApplyPub = driveApplyTopic.publish();
        this.driveApplySub = driveApplyTopic.subscribe(false);
        driveApplyPub.set(false); // Initial value

        var steerApplyTopic = table.getBooleanTopic("Steer Apply");
        this.steerApplyPub = steerApplyTopic.publish();
        this.steerApplySub = steerApplyTopic.subscribe(false);
        steerApplyPub.set(false); // Initial value
    }

    public void periodic() {
        drivePidTuning.periodic();
        steerPidTuning.periodic();

        if (driveApplySub.get()) {
            drivePidConfig.setDirty();
            driveApplyPub.set(false); // Reset the button
        }

        if (steerApplySub.get()) {
            steerPidConfig.setDirty();
            steerApplyPub.set(false); // Reset the button
        }
    }
}
