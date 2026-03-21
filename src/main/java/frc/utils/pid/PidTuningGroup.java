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
    private final DoublePublisher kVPub;
    private final DoublePublisher kSPub;

    private final DoubleSubscriber kpSub;
    private final DoubleSubscriber kiSub;
    private final DoubleSubscriber kdSub;
    private final DoubleSubscriber kVSub;
    private final DoubleSubscriber kSSub;


    private final PidConfig config;

    public PidTuningGroup(DashboardTuner tuner, String prefix, PidConfig config) {
        this.config = config;

        String p = prefix.isEmpty() ? "" : prefix + "/";
        var kpTopic = tuner.getTable().getDoubleTopic(p + "kP");
        var kiTopic = tuner.getTable().getDoubleTopic(p + "kI");
        var kdTopic = tuner.getTable().getDoubleTopic(p + "kD");
        var kVTopic = tuner.getTable().getDoubleTopic(p + "kV");
        var kSTopic = tuner.getTable().getDoubleTopic(p + "kS");

        kpPub = kpTopic.publish();
        kiPub = kiTopic.publish();
        kdPub = kdTopic.publish();
        kVPub = kVTopic.publish();
        kSPub = kSTopic.publish();

        kpSub = kpTopic.subscribe(config.kP);
        kiSub = kiTopic.subscribe(config.kI);
        kdSub = kdTopic.subscribe(config.kD);
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
        kVPub.set(config.kV);
        kSPub.set(config.kS);
    }

    public void updateConfig(PidConfig config) {
        double kP = kpSub.get();
        double kI = kiSub.get();
        double kD = kdSub.get();
        double kV = kVSub.get();
        double kS = kSSub.get();

        // Echo any Elastic-changed values back through the server publisher.
        // Without this, the server subscriber sees its own stale initial value
        // rather than the value the user set in the Elastic slider.
        if (kP != config.kP) kpPub.set(kP);
        if (kI != config.kI) kiPub.set(kI);
        if (kD != config.kD) kdPub.set(kD);
        if (kV != config.kV) kVPub.set(kV);
        if (kS != config.kS) kSPub.set(kS);

        config.withKP(kP).withKI(kI).withKD(kD).withKV(kV).withKS(kS);
    }
}
