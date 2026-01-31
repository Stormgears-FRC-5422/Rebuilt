package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Shooter;

import frc.robot.elastic.TestShooterTuner;
import frc.robot.subsystems.drive.config.PidConfig;

public class KrakenShooter extends SubsystemBase {
    public final TalonFX motor = new TalonFX(1);

    private final PidConfig pid;

    int num =0;

    private final TestShooterTuner tuner;
    
    private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);

    double maxrps = 100.0;
    
    public KrakenShooter() {
        // Initialize PID configs
        pid = new PidConfig()
                .withKP(Shooter.driveKP)
                .withKI(Shooter.driveKI)
                .withKD(Shooter.driveKD)
                .withKV(Shooter.driveKV)
                .withKMinMax(Shooter.driveKMin, Shooter.driveKMax);

        tuner = new TestShooterTuner("Shooter PID", pid);

        configMotor();
    }

    @Override
    public void periodic() {
        tuner.periodic(); 

        if (pid.isDirty() || num++ % 500 == 0) {
            configMotor();
            pid.clearDirty();
            System.out.println(pid);
        } 
        Logger.recordOutput("vel",motor.getVelocity().getValueAsDouble());

    }

    void configMotor() {
        Slot0Configs slot0 = new Slot0Configs();

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

    public void setVelocityClosedLoop(double velocityRps) {
        // percent = Math.max(-1.0, Math.min(1.0, percent)); 
        // double targetRps = percent * maxrps;

        motor.setControl(m_velocityVoltage.withVelocity(velocityRps));
        
        
    }

    public void stop() {
        motor.set(0.0); 
    }
}
