package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.utils.StormSubsystem;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants.Shooter;

import frc.utils.pid.PidConfig;

public class KrakenShooter extends StormSubsystem {
    public final TalonFX motor = new TalonFX(1);

    private final PidConfig pid;
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);
    int num = 0;

    public KrakenShooter() {
        // Initialize PID configs
        pid = new PidConfig()
                .withKP(Shooter.shootKP)
                .withKI(Shooter.shootKI)
                .withKD(Shooter.shootKD)
                .withKV(Shooter.shootKV)
                .withKS(Shooter.shootKS)
                .withKMinMax(Shooter.shootKMin, Shooter.shootKMax);

        configMotor();
    }

    @Override
    public void periodic() {
        if (pid.isDirty() || num++ % 500 == 0) {
            configMotor();
            pid.clearDirty();
            console(String.valueOf(pid),1);
        }

        Logger.recordOutput("vel", motor.getVelocity().getValueAsDouble());
    }

    public PidConfig getPidConfig() {
        return pid;
    }

    public void setVelocityClosedLoop(double velocityRps) {
        motor.setControl(velocityVoltage.withVelocity(velocityRps));
    }
    
    public void setTargetVelocity(double targetRps) {
        setVelocityClosedLoop(targetRps);
    }

    public void stop() {
        motor.set(0.0);
    } 

    void configMotor() {
        Slot0Configs slot0 = new Slot0Configs();

        // From phoenix 6 examples
        // Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
        // configs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
        // configs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
        // configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
        // configs.Slot0.kI = 0; // No output for integrated error
        // configs.Slot0.kD = 0; // No output for error derivative

        slot0.kP = pid.kP;
        slot0.kI = pid.kI;
        slot0.kD = pid.kD;
        slot0.kS = pid.kS;
        slot0.kV = pid.kV;

        motor.getConfigurator().apply(slot0);

        MotorOutputConfigs out = new MotorOutputConfigs();
        out.PeakForwardDutyCycle = pid.kMax;
        out.PeakReverseDutyCycle = pid.kMin;
        motor.getConfigurator().apply(out);
    }
}
