package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.elastic.TestShooterTuner;
import frc.robot.subsystems.drive.config.PidConfig;

public class KrakenPID extends SubsystemBase {
    private final TalonFX motor = new TalonFX(1);

    private final PidConfig pid = new PidConfig()
        .withKMinMax(-1.0, 1.0);

    private final TestShooterTuner tuner = new TestShooterTuner("Kraken PID", pid);

    private final VelocityVoltage velReq = new VelocityVoltage(0).withSlot(0);

    double maxrps = 100.0;

    @Override
    public void periodic() {
        tuner.periodic(); 

        if (pid.isDirty()) {
            Slot0Configs slot0 = new Slot0Configs();
            slot0.kP = pid.kP;
            slot0.kI = pid.kI;
            slot0.kD = pid.kD;
            slot0.kS = pid.kS;
            slot0.kV = pid.kV;
            slot0.kA = pid.kA;

            motor.getConfigurator().apply(slot0);

            MotorOutputConfigs out = new MotorOutputConfigs();
            out.PeakForwardDutyCycle = pid.kMax;
            out.PeakReverseDutyCycle = pid.kMin;
            motor.getConfigurator().apply(out);

            pid.clearDirty();
        }
    }

    public void setPercentVelocityClosedLoop(double percent) {
        percent = Math.max(-1.0, Math.min(1.0, percent));
        double targetRps = percent * maxrps;

        motor.setControl(velReq.withVelocity(targetRps));
    }

    public void stop() {
        motor.set(0.0); 
    }
}
