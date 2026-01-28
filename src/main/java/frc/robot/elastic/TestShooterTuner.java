package frc.robot.elastic;

import frc.robot.subsystems.drive.config.PidConfig;
import frc.utils.configfile.DashboardTuner;
import frc.utils.configfile.PidTuningGroup;

public class TestShooterTuner extends DashboardTuner {

    private final PidTuningGroup shooterPIDTuning;

    public TestShooterTuner(String name, PidConfig shootPIDConfig) {
        super(name);
        this.shooterPIDTuning = new PidTuningGroup(this, "shooter", shootPIDConfig);
    }

    public void periodic() {
        shooterPIDTuning.periodic();
    }
}
