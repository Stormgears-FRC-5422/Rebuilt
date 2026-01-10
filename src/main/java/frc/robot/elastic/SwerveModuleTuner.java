package frc.robot.elastic;

import frc.robot.subsystems.drive.config.PidConfig;
import frc.utils.configfile.DashboardTuner;
import frc.utils.configfile.PidTuningGroup;

public class SwerveModuleTuner extends DashboardTuner {

    private final PidTuningGroup drivePidTuning;
    private final PidTuningGroup steerPidTuning;

    public SwerveModuleTuner(String name, PidConfig drivePidConfig, PidConfig steerPidConfig) {
        super(name);
        this.drivePidTuning = new PidTuningGroup(this, "Drive", drivePidConfig);
        this.steerPidTuning = new PidTuningGroup(this, "Steer", steerPidConfig);
    }

    public void periodic() {
        drivePidTuning.periodic();
        steerPidTuning.periodic();
    }
}
