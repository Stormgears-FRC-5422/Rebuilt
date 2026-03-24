package frc.utils.pid;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;

/**
 * Groups related NetworkTable entries for PID tuning. Each group manages
 * P, I, D, A, V, S values by default. Optional gains (kG, kMin/kMax) can
 * be added via fluent {@code with*()} methods.
 */
public final class PidTuningGroup {

    private final DoublePublisher kpPub;
    private final DoublePublisher kiPub;
    private final DoublePublisher kdPub;
    private final DoublePublisher kAPub;
    private final DoublePublisher kVPub;
    private final DoublePublisher kSPub;

    private final DoubleSubscriber kpSub;
    private final DoubleSubscriber kiSub;
    private final DoubleSubscriber kdSub;
    private final DoubleSubscriber kASub;
    private final DoubleSubscriber kVSub;
    private final DoubleSubscriber kSSub;

    // Optional — null until with*() called
    private DoublePublisher kGPub, kMinPub, kMaxPub;
    private DoubleSubscriber kGSub, kMinSub, kMaxSub;

    private final NetworkTable table;
    private final String prefix;
    private final PidConfig config;

    public PidTuningGroup(DashboardTuner tuner, String prefix, PidConfig config) {
        this.config = config;
        this.table = tuner.getTable();
        this.prefix = prefix.isEmpty() ? "" : prefix + "/";

        var kpTopic = table.getDoubleTopic(this.prefix + "kP");
        var kiTopic = table.getDoubleTopic(this.prefix + "kI");
        var kdTopic = table.getDoubleTopic(this.prefix + "kD");
        var kATopic = table.getDoubleTopic(this.prefix + "kA");
        var kVTopic = table.getDoubleTopic(this.prefix + "kV");
        var kSTopic = table.getDoubleTopic(this.prefix + "kS");

        kpPub = kpTopic.publish();
        kiPub = kiTopic.publish();
        kdPub = kdTopic.publish();
        kAPub = kATopic.publish();
        kVPub = kVTopic.publish();
        kSPub = kSTopic.publish();

        kpSub = kpTopic.subscribe(config.kP);
        kiSub = kiTopic.subscribe(config.kI);
        kdSub = kdTopic.subscribe(config.kD);
        kASub = kATopic.subscribe(config.kA);
        kVSub = kVTopic.subscribe(config.kV);
        kSSub = kSTopic.subscribe(config.kS);

        // Set initial values
        doPublishConfig(config);
    }

    /** Add gravity feedforward (kG) to the dashboard. */
    public PidTuningGroup withGravity() {
        var topic = table.getDoubleTopic(prefix + "kG");
        kGPub = topic.publish();
        kGSub = topic.subscribe(config.kG);
        kGPub.set(config.kG);
        return this;
    }

    /** Add output limits (kMin, kMax) to the dashboard. */
    public PidTuningGroup withOutputLimits() {
        var kMinTopic = table.getDoubleTopic(prefix + "kMin");
        var kMaxTopic = table.getDoubleTopic(prefix + "kMax");
        kMinPub = kMinTopic.publish();
        kMaxPub = kMaxTopic.publish();
        kMinSub = kMinTopic.subscribe(config.kMin);
        kMaxSub = kMaxTopic.subscribe(config.kMax);
        kMinPub.set(config.kMin);
        kMaxPub.set(config.kMax);
        return this;
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
        kVPub.set(config.kV);
        kSPub.set(config.kS);
        if (kGPub != null) kGPub.set(config.kG);
        if (kMinPub != null) kMinPub.set(config.kMin);
        if (kMaxPub != null) kMaxPub.set(config.kMax);
    }

    public void updateConfig(PidConfig config) {
        double kP = kpSub.get();
        double kI = kiSub.get();
        double kD = kdSub.get();
        double kA = kASub.get();
        double kV = kVSub.get();
        double kS = kSSub.get();

        // Echo any Elastic-changed values back through the server publisher.
        // Without this, the server subscriber sees its own stale initial value
        // rather than the value the user set in the Elastic slider.
        if (kP != config.kP) kpPub.set(kP);
        if (kI != config.kI) kiPub.set(kI);
        if (kD != config.kD) kdPub.set(kD);
        if (kA != config.kA) kAPub.set(kA);
        if (kV != config.kV) kVPub.set(kV);
        if (kS != config.kS) kSPub.set(kS);

        config.withKP(kP).withKI(kI).withKD(kD).withKA(kA).withKV(kV).withKS(kS);

        if (kGSub != null) {
            double kG = kGSub.get();
            if (kG != config.kG) kGPub.set(kG);
            config.withKG(kG);
        }
        if (kMinSub != null && kMaxSub != null) {
            double kMin = kMinSub.get();
            double kMax = kMaxSub.get();
            if (kMin != config.kMin) kMinPub.set(kMin);
            if (kMax != config.kMax) kMaxPub.set(kMax);
            config.withKMinMax(kMin, kMax);
        }
    }
}
