package frc.robot.subsystems.ballControl;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.utils.StormSubsystem;

import static edu.wpi.first.units.Units.Degrees;

public class Intake extends StormSubsystem {

    private final TalonFX updownIntakeMotor;
    // private final TalonFX rollerIntakeMotor;

    private final MotionMagicVoltage mmRequest;

    private double targetDeg = 0.0;

    public Intake() {
        updownIntakeMotor = new TalonFX(20);
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        mmRequest = new MotionMagicVoltage(0.0);

        config.Feedback.SensorToMechanismRatio = 1.0;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        config.Slot0.kP = 1.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kG = 0.5;

        config.MotionMagic.MotionMagicCruiseVelocity = 1.0; // rot /s
        config.MotionMagic.MotionMagicAcceleration = 2.0; // rot/s^2

        updownIntakeMotor.getConfigurator().apply(config);
        updownIntakeMotor.setNeutralMode(NeutralModeValue.Brake);

        updownIntakeMotor.getConfigurator().setPosition(0.0);
    }

    public void setAngleDegrees(double degrees) {
        targetDeg = degrees;

        double accRotations = degrees / 360.0;

        updownIntakeMotor.setControl(mmRequest.withPosition(accRotations));
    }

    public void moveDeltaDegrees(double deltaDeg) {
        setAngleDegrees(getAngleDegrees() + deltaDeg);
    }

    public void zeroHere() {
        updownIntakeMotor.getConfigurator().setPosition(0.0);
        targetDeg = 0.0;
    }

    public double getAngleDegrees() {
        return updownIntakeMotor.getPosition().getValue().in(Degrees);
    }
}
