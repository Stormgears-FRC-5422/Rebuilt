package frc.utils.configfile;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import frc.robot.subsystems.drive.config.PidConfig;

/**
 * Groups related NetworkTable entries for PID tuning. Each group manages the P,
 * I, D, Max, and V values for a PID configuration.
 */
public final class PidTuningGroup {

    private final DoublePublisher kpPub;
    private final DoublePublisher kiPub;
    private final DoublePublisher kdPub;
    private final DoublePublisher kMinPub;
    private final DoublePublisher kMaxPub;
    private final DoublePublisher kVPub;

    private final DoubleSubscriber kpSub;
    private final DoubleSubscriber kiSub;
    private final DoubleSubscriber kdSub;
    private final DoubleSubscriber kMinSub;
    private final DoubleSubscriber kMaxSub;
    private final DoubleSubscriber kVSub;

    private final PidConfig config;

    public PidTuningGroup(DashboardTuner tuner, String prefix, PidConfig config) {
        this.config = config;

        var kpTopic = tuner.getTable().getDoubleTopic(prefix + "/kP");
        var kiTopic = tuner.getTable().getDoubleTopic(prefix + "/kI");
        var kdTopic = tuner.getTable().getDoubleTopic(prefix + "/kD");
        var kMinTopic = tuner.getTable().getDoubleTopic(prefix + "/kMin");
        var kMaxTopic = tuner.getTable().getDoubleTopic(prefix + "/kMax");
        var kVTopic = tuner.getTable().getDoubleTopic(prefix + "/kV");

        kpPub = kpTopic.publish();
        kiPub = kiTopic.publish();
        kdPub = kdTopic.publish();
        kMinPub = kMinTopic.publish();
        kMaxPub = kMaxTopic.publish();
        kVPub = kVTopic.publish();

        kpSub = kpTopic.subscribe(config.kP);
        kiSub = kiTopic.subscribe(config.kI);
        kdSub = kdTopic.subscribe(config.kD);
        kMinSub = kMinTopic.subscribe(config.kMin);
        kMaxSub = kMaxTopic.subscribe(config.kMax);
        kVSub = kVTopic.subscribe(config.kV);

        // Set initial values
        doPublishConfig(config);
    }

    public void periodic() {
        updateConfig(config);
    }

    public void publishConfig(PidConfig config) {
        doPublishConfig(config);
    }

    private void doPublishConfig(PidConfig config) {
        kpPub.set(config.kP);
        kiPub.set(config.kI);
        kdPub.set(config.kD);
        kMinPub.set(config.kMin);
        kMaxPub.set(config.kMax);
        kVPub.set(config.kV);
    }

    public void updateConfig(PidConfig config) {
        config.withKP(kpSub.get())
                .withKI(kiSub.get())
                .withKD(kdSub.get())
                .withKMinMax(kMinSub.get(), kMaxSub.get())
                .withKV(kVSub.get());
    }
}
