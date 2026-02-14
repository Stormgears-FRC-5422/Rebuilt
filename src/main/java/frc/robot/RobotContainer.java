// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Toggles;
import frc.robot.commands.JoyStickDrive;
import frc.robot.commands.SetShooterVelocity;
import frc.robot.elastic.ElasticMotorController;
import frc.robot.elastic.SingleMotorTuner;
import frc.robot.joysticks.IllegalJoystickTypeException;
import frc.robot.joysticks.RebuiltJoystick;
import frc.robot.joysticks.RebuiltJoystickFactory;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ballControl.Intake;
import frc.robot.subsystems.ballControl.*;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.drive.DrivetrainFactory;
import frc.robot.subsystems.drive.IllegalDriveTypeException;

public class RobotContainer {
    // **********
    // Control
    // **********
    RebuiltJoystick joystick;
    @SuppressWarnings("unused")
    private DrivetrainBase drivetrain;

    @SuppressWarnings("unused")
    private VisionSubsystem vision;

    private SingleMotorTuner shooterTuner;
    private ShooterSubsystem krakenShooter;
    private Intake intake;

    //elastic control
    private final ElasticMotorController elasticMotorController = new ElasticMotorController();

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
            joystick = RebuiltJoystickFactory.getInstance(Constants.ButtonBoard.driveJoystick, Constants.ButtonBoard.driveJoystickPort);

            if (Toggles.useDrive) {
                console("Create drive type " + Constants.Drive.driveType);
                drivetrain = DrivetrainFactory.getInstance(Constants.Drive.driveType, Constants.Drive.driveSubtype);
            }

            if (Toggles.useVision && Toggles.useDrive && drivetrain != null) {
                console("Create vision");
                vision = new VisionSubsystem(drivetrain, "limelight");
            }

            if (Toggles.useShooter) {
                console("Create shooter");
                krakenShooter = new ShooterSubsystem();
                intake = new Intake();
                if (Toggles.usePidTuner) {
                    shooterTuner = new SingleMotorTuner("Shooter Tuner", krakenShooter.getPidConfig());
                }
            }

            if (Toggles.useLights) {
                lights = new Lights();
            }
            configureBindings();
            console("[DONE] RobotContainer");
        } catch (IllegalDriveTypeException e) {
            console("Error: Invalid drive type specified: " + e.getMessage());
        } catch (RuntimeException e) {
            console("Error initializing RobotContainer: " + e.getMessage());
        } catch (IllegalJoystickTypeException e) {
            console("Error: Invalid joystick type specified: " + e.getMessage());
        }
    }

    private void configureBindings() {
        Trigger elasticEnabled = new Trigger(() -> elasticMotorController.enabled.get());

        if (intake != null) {
            elasticEnabled.onTrue(
                Commands.runOnce(() -> intake.setAngleDegrees(360.0), intake)
            );

            elasticEnabled.onFalse(
                Commands.runOnce(() -> intake.setAngleDegrees(0.0), intake)
            );
        }
        console("[Init] configureBindings");

        if (Toggles.useDrive) {
            JoyStickDrive driveWithJoystick = new JoyStickDrive(drivetrain, joystick);

            drivetrain.setDefaultCommand(driveWithJoystick);

            new Trigger(() -> joystick.resetGyro())
                .onTrue(new InstantCommand(() -> drivetrain.resetOrientation()));

                //this code is only for running the command RunShoot
        //     new Trigger(() -> elasticMotorController.enabled.get())
        //         .whileTrue(new RunShooter(krakenShooter,() -> -slider(elasticMotorController.motor1.get(), -1.0, 1.0) * 100.0
        // )); 
        
            // Need a test here to allow this based on the drive type.
//            HoldSwerveModuleStates holdCommand = new HoldSwerveModuleStates(drivetrain, MetersPerSecond.of(0.5), Degrees.of(90));
//            new Trigger(() -> joystick.holdSwerveState()).whileTrue(holdCommand);
        }

        console("[DONE] configureBindings");
    }
    private static double slider(double x, double low, double high) {
        return Math.max(low, Math.min(high, x));
    }

    public void periodic() {
        if (shooterTuner != null) {
            shooterTuner.periodic();
        }
    }

    /**
     * Updates alliance state from DriverStation/FMS.
     * Respects debug override when Constants.Debug.debug is true and defaultAlliance is set.
     * When alliance becomes available (or changes), calls onAllianceUpdated().
     */
    public void updateAlliance() {
        RobotState robotState = RobotState.getInstance();
        RobotState.StateAlliance newAlliance = RobotState.StateAlliance.MISSING;
        String defaultAlliance = Constants.Debug.defaultAlliance.toLowerCase();

        if (!Constants.Debug.debug || defaultAlliance.equals("auto")) {
            // Use DriverStation/FMS
            DriverStation.refreshData();
            Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

            if (alliance.isPresent()) {
                newAlliance = alliance.get() == DriverStation.Alliance.Blue
                    ? RobotState.StateAlliance.BLUE
                    : RobotState.StateAlliance.RED;
            }
        } else {
            // Debug override active
            newAlliance = defaultAlliance.equals("blue")
                ? RobotState.StateAlliance.BLUE
                : RobotState.StateAlliance.RED;
        }

        RobotState.StateAlliance previousAlliance = robotState.getAlliance();

        // Call onAllianceUpdated when alliance first becomes known or changes
        if (newAlliance != RobotState.StateAlliance.MISSING
            && (previousAlliance == RobotState.StateAlliance.MISSING || previousAlliance != newAlliance)) {
            robotState.setAlliance(newAlliance);
            onAllianceUpdated();
        }
    }

    /**
     * Called when alliance is first set or changes.
     * Resets initial pose and performs any other alliance-dependent initialization.
     */
    private void onAllianceUpdated() {
        console("Alliance updated to " + RobotState.getInstance().getAlliance());
        resetInitialPose();
    }

    /**
     * Resets the drivetrain pose to the initial pose from debug config,
     * remapped for the current alliance.
     */
    public void resetInitialPose() {
        RobotState robotState = RobotState.getInstance();
        if (!Toggles.useDrive || drivetrain == null) {
            return;
        }

        Pose2d initialPose;
        if (Constants.Debug.debug && !robotState.getAlliance().equals(RobotState.StateAlliance.MISSING)) {
            // Use debug pose settings, remapped for alliance
            Pose2d bluePose = new Pose2d(
                Constants.Debug.initPoseX,
                Constants.Debug.initPoseY,
                Rotation2d.fromDegrees(Constants.Debug.initPoseDegrees)
            );
            initialPose = RebuiltField.remapPose(bluePose, robotState.getAlliance());
        } else {
            // Default pose - AutoChooser should set the pose itself for matches
            initialPose = new Pose2d();
        }
        drivetrain.setPose(initialPose);
    }

    private void console(String message) {
        System.out.println("RobotContainer : " + message);
    }
}
