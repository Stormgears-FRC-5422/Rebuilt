// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Toggles;
import frc.robot.commands.HoldSwerveModuleStates;
import frc.robot.commands.JoyStickDrive;
import frc.robot.elastic.SwerveModuleTuner;
import frc.robot.joysticks.RebuiltJoystick;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.drive.DrivetrainFactory;
import frc.robot.subsystems.drive.IllegalDriveTypeException;
import frc.robot.subsystems.drive.config.SwerveModuleConstants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class RobotContainer {
    // **********
    // Control
    // **********
    RebuiltJoystick joystick;
    @SuppressWarnings("unused")
    private DrivetrainBase drivetrain;
    private SwerveModuleTuner tuner;

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

            if (Toggles.usePidTuner) {
                tuner = new SwerveModuleTuner("PID Tuning", SwerveModuleConstants.drivePidConfig, SwerveModuleConstants.steerPidConfig);
            }

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

    public void periodic() {
        if (tuner != null) {
            tuner.periodic();
        }
    }

    private void console(String message) {
        System.out.println("RobotContainer : " + message);
    }
}
