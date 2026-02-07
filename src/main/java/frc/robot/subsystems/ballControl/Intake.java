package frc.robot.subsystems.ballControl;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants.Shooter;
import frc.utils.StormSubsystem;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;


public class Intake extends StormSubsystem {
    
    // TalonFX updownIntakeMotor;
    // TalonFX rollerIntakeMotor;
    
    SparkMax updownIntakeMotor;
    RelativeEncoder updownEncoder;
    SparkClosedLoopController updownPID;
    ArmFeedforward feedforward;

    double targetPositionRotations = 0.0;


    public Intake() {
        // updownIntakeMotor = new TalonFX(Shooter.intakeUpDownID);
        // rollerIntakeMotor = new TalonFX(Shooter.intakeRollerID);

        // Slot0Configs slot0 = new Slot0Configs();
        // slot0.kP = 0;
        // slot0.kI = 0.0;
        // slot0.kD = 0.0;
        // slot0.kG = 0.1;
        // slot0.GravityType = GravityTypeValue.Arm_Cosine;
    
        // TalonFXConfiguration config = new TalonFXConfiguration();
        // config.Slot0 = slot0;
        
        // need gear ratio
        // config.Feedback.SensorToMechanismRatio=;
            
        // config.withMotorOutput(new MotorOutputConfigs()
        // .withInverted(InvertedValue.Clockwise_Positive)
        // ); in case it starts moving the wrong way thru the robot
        // updownIntakeMotor.getConfigurator().apply(config);

        // updownIntakeMotor.setPosition(0.25); // p sure this is in rotations, so 0.25 = 90degrees
    
        updownIntakeMotor = new SparkMax(Shooter.intakeUpDownID, MotorType.kBrushless);

        feedforward = new ArmFeedforward(
            0.0,  // kS
            0.1,  // kG 
            0.0,  // kV
            0.0   // kA
        );
        
        SparkMaxConfig config = new SparkMaxConfig();

        ClosedLoopConfig cl = config.closedLoop;

        cl.pid(1.0, 0.0, 0.1, ClosedLoopSlot.kSlot0);

        updownIntakeMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }  

    public void goToWhatever(double targetPosition) {
        this.targetPositionRotations = targetPosition;

        // updownIntakeMotor.setControl(new PositionVoltage(targetPosition));    
    }
    
    @Override
    public void periodic() {
        double positionRadians = updownEncoder.getPosition() * (2.0*Math.PI);

        double ffVolts = feedforward.calculate(positionRadians, 0.0);

        updownPID.setReference(targetPositionRotations, SparkMax.ControlType.kPosition,ClosedLoopSlot.kSlot0,ffVolts);

    }
}
