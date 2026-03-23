package frc.robot.elastic;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import frc.utils.pid.PidConfig;
import frc.utils.pid.DashboardTuner;
import frc.utils.pid.PidTuningGroup;

public class SingleMotorTuner extends DashboardTuner {

    private final PidTuningGroup shooterPIDTuning;
    private final BooleanPublisher applyPub;
    private final BooleanSubscriber applySub;
    private final PidConfig pidConfig;

    public SingleMotorTuner(String name, PidConfig shootPIDConfig) {
        super(name);
        this.pidConfig = shootPIDConfig;
        this.shooterPIDTuning = new PidTuningGroup(this, "", shootPIDConfig);

        // Create apply button at root level of the tuner
        var applyTopic = table.getBooleanTopic("Apply");
        this.applyPub = applyTopic.publish();
        this.applySub = applyTopic.subscribe(false);
        applyPub.set(false); // Initial value
    }

    /** Add gravity feedforward (kG) to the dashboard. */
    public SingleMotorTuner withGravity() {
        shooterPIDTuning.withGravity();
        return this;
    }

    /** Add output limits (kMin, kMax) to the dashboard. */
    public SingleMotorTuner withOutputLimits() {
        shooterPIDTuning.withOutputLimits();
        return this;
    }

    public void periodic() {
        shooterPIDTuning.periodic();

        if (applySub.get()) {
            pidConfig.setDirty();
            applyPub.set(false); // Reset the button
        }
    }
}
