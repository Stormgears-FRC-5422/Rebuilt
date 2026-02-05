import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants.Shooter;
import frc.utils.StormSubsystem;

public class Intake extends StormSubsystem {
    
    TalonFX updownIntakeMotor;
    TalonFX rollerIntakeMotor;

    public Intake() {
        updownIntakeMotor = new TalonFX(Shooter.intakeUpDownID);
        rollerIntakeMotor = new TalonFX(Shooter.intakeRollerID);

        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = 0;
        slot0.kI = 0.0;
        slot0.kD = 0.0;
        slot0.kG = 0.1;
        slot0.GravityType = GravityTypeValue.Arm_Cosine;
    
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0 = slot0;
        
        // config.Feedback.SensorToMechanismRatio=;
            
        // config.withMotorOutput(new MotorOutputConfigs()
        // .withInverted(InvertedValue.Clockwise_Positive)
        // ); in case it starts moving the wrong way thru the robot
        updownIntakeMotor.getConfigurator().apply(config);

        updownIntakeMotor.setPosition(0.25); // p sure this is in rotations, so 0.25 = 90degrees
    }  

    public void goToWhatever(double targetPosition) {
        updownIntakeMotor.setControl(new PositionVoltage(targetPosition));    
    }
}
