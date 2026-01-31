package frc.robot.elastic;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import frc.robot.subsystems.drive.config.PidConfig;
import frc.utils.configfile.DashboardTuner;
import frc.utils.configfile.PidTuningGroup;

public class SingleMotorTuner extends DashboardTuner {

    private final PidTuningGroup shooterPIDTuning;
    private final BooleanPublisher applyPub;
    private final BooleanSubscriber applySub;
    private final PidConfig pidConfig;

    public SingleMotorTuner(String name, PidConfig shootPIDConfig) {
        super(name);
        this.pidConfig = shootPIDConfig;
        this.shooterPIDTuning = new PidTuningGroup(this, "PID", shootPIDConfig);

        // Create apply button at root level of the tuner
        var applyTopic = table.getBooleanTopic("Apply");
        this.applyPub = applyTopic.publish();
        this.applySub = applyTopic.subscribe(false);
        applyPub.set(false); // Initial value
    }

    public void periodic() {
        shooterPIDTuning.periodic();

        if (applySub.get()) {
            pidConfig.setDirty();
            applyPub.set(false); // Reset the button
        }
    }
}
