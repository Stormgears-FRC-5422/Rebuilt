// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Toggles;
import frc.robot.commands.HoldSwerveModuleStates;
import frc.robot.commands.JoyStickDrive;
import frc.robot.elastic.TestShooterTuner;
import frc.robot.elastic.ElasticMotorController;
import frc.robot.joysticks.RebuiltJoystick;
import frc.robot.subsystems.KrakenShooter;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.drive.DrivetrainFactory;
import frc.robot.subsystems.drive.IllegalDriveTypeException;
import frc.robot.subsystems.drive.config.SwerveModuleConstants;
import frc.robot.elastic.ElasticMotorController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DriverStation;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;

public class RobotContainer {
    // **********
    // Control
    // **********
    RebuiltJoystick joystick;
    @SuppressWarnings("unused")
    private DrivetrainBase drivetrain;

    // private TestShooterTuner tuner;
    //elastic control
    private final ElasticMotorController elasticMotorController = new ElasticMotorController();
    // private final SparkMax motor1 = new SparkMax(1, MotorType.kBrushless);

    private final KrakenShooter krakenShooter = new KrakenShooter();

    // private final SparkMax motor2 = new SparkMax(2, MotorType.kBrushless);

    // **********
    // Commands
    // **********
    // (no pre-created commands)
    @SuppressWarnings("unused")
    private Lights lights;
    public RobotContainer() {
        try {
            console("[Init] RobotContainer");
            console("Making joystick!");
            joystick = new RebuiltJoystick(Constants.ButtonBoard.driveJoystickPort);
            if (Toggles.useDrive) {
                console("Create drive type " + Constants.Drive.driveType);
                drivetrain = DrivetrainFactory.getInstance(Constants.Drive.driveType, "");
            }
            if (Toggles.useLights) {
                lights = new Lights();
            }
            // if (Toggles.usePidTuner) {
            //     tuner = new TestShooterTuner("PID Tuning", SwerveModuleConstants.drivePidConfig);
            // }
            configureBindings();
            console("[DONE] RobotContainer");
        } catch (IllegalDriveTypeException e) {
            console("Error: Invalid drive type specified: " + e.getMessage());
        } catch (RuntimeException e) {
            console("Error initializing RobotContainer: " + e.getMessage());
        }
    }
    private void configureBindings() {
        console("[Init] configureBindings");
        if (Toggles.useDrive) {
            JoyStickDrive driveWithJoystick = new JoyStickDrive(drivetrain, joystick);
            HoldSwerveModuleStates holdCommand = new HoldSwerveModuleStates(drivetrain, MetersPerSecond.of(0.5), Degrees.of(90));
            drivetrain.setDefaultCommand(driveWithJoystick);
            new Trigger(() -> joystick.holdSwerveState()).whileTrue(holdCommand);
        }
        console("[DONE] configureBindings");
    }
    private static double slider(double x, double low, double high) {
        return Math.max(low, Math.min(high, x));
    }

    public void periodic() {
        // if (tuner != null) {
        //     tuner.periodic();
        // }
        Options.periodic();

        if (elasticMotorController.enabled.get()) {
            // double percent = slider(motorTestTuner.motor1.get(), -1.0, 1.0);
            krakenShooter.setVelocityClosedLoop(-elasticMotorController.motor1.get()*50);
        } else {
            krakenShooter.stop();
        }

        // if (motorTestTuner.enabled.get()) {
        // double m1 = slider(motorTestTuner.motor1.get(), -1.0, 1.0);
        // // double m2 = slider(motorTestTuner.motor2.get(), -1.0, 1.0);

        // motor1.set(m1);
        // // motor2.set(m2);
        // } else {
        //     motor1.set(0.0);
        //     // motor2.set(0.0);
        // }
    }
    private void console(String message) {
        System.out.println("RobotContainer : " + message);
    }
}