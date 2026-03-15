package frc.utils.pid;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;

/**
 * Groups related NetworkTable entries for PID tuning. Each group manages the P,
 * I, D, Max, and V values for a PID configuration.
 */
public final class PidTuningGroup {

    private final DoublePublisher kpPub;
    private final DoublePublisher kiPub;
    private final DoublePublisher kdPub;
    private final DoublePublisher kAPub;
    private final DoublePublisher kGPub;
    private final DoublePublisher kMinPub;
    private final DoublePublisher kMaxPub;
    private final DoublePublisher kVPub;
    private final DoublePublisher kSPub;

    private final DoubleSubscriber kpSub;
    private final DoubleSubscriber kiSub;
    private final DoubleSubscriber kdSub;
    private final DoubleSubscriber kASub;
    private final DoubleSubscriber kGSub;
    private final DoubleSubscriber kMinSub;
    private final DoubleSubscriber kMaxSub;
    private final DoubleSubscriber kVSub;
    private final DoubleSubscriber kSSub;


    private final PidConfig config;

    public PidTuningGroup(DashboardTuner tuner, String prefix, PidConfig config) {
        this.config = config;

        String p = prefix.isEmpty() ? "" : prefix + "/";
        var kpTopic = tuner.getTable().getDoubleTopic(p + "kP");
        var kiTopic = tuner.getTable().getDoubleTopic(p + "kI");
        var kdTopic = tuner.getTable().getDoubleTopic(p + "kD");
        var kATopic = tuner.getTable().getDoubleTopic(p + "kA");
        var kGTopic = tuner.getTable().getDoubleTopic(p + "kG");
        var kMinTopic = tuner.getTable().getDoubleTopic(p + "kMin");
        var kMaxTopic = tuner.getTable().getDoubleTopic(p + "kMax");
        var kVTopic = tuner.getTable().getDoubleTopic(p + "kV");
        var kSTopic = tuner.getTable().getDoubleTopic(p + "kS");

        kpPub = kpTopic.publish();
        kiPub = kiTopic.publish();
        kdPub = kdTopic.publish();
        kAPub = kATopic.publish();
        kGPub = kGTopic.publish();
        kMinPub = kMinTopic.publish();
        kMaxPub = kMaxTopic.publish();
        kVPub = kVTopic.publish();
        kSPub = kSTopic.publish();

        kpSub = kpTopic.subscribe(config.kP);
        kiSub = kiTopic.subscribe(config.kI);
        kdSub = kdTopic.subscribe(config.kD);
        kASub = kATopic.subscribe(config.kA);
        kGSub = kGTopic.subscribe(config.kG);
        kMinSub = kMinTopic.subscribe(config.kMin);
        kMaxSub = kMaxTopic.subscribe(config.kMax);
        kVSub = kVTopic.subscribe(config.kV);
        kSSub = kSTopic.subscribe(config.kS);

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
        kAPub.set(config.kA);
        kGPub.set(config.kG);
        kMinPub.set(config.kMin);
        kMaxPub.set(config.kMax);
        kVPub.set(config.kV);
        kSPub.set(config.kS);
    }

    public void updateConfig(PidConfig config) {
        config.withKP(kpSub.get())
                .withKI(kiSub.get())
                .withKD(kdSub.get())
                .withKA(kASub.get())
                .withKG(kGSub.get())
                .withKMinMax(kMinSub.get(), kMaxSub.get())
                .withKV(kVSub.get())
                .withKS(kSSub.get());
    }
}
