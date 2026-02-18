// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Toggles;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.HoldSwerveModuleStates;
import frc.robot.commands.JoyStickDrive;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.RunIntake;

import frc.robot.elastic.ElasticMotorController;
import frc.robot.elastic.SingleMotorTuner;
import frc.robot.joysticks.IllegalJoystickTypeException;
import frc.robot.joysticks.RebuiltJoystick;
import frc.robot.joysticks.RebuiltJoystickFactory;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.ShooterHopper;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.drive.DrivetrainFactory;
import frc.robot.subsystems.drive.IllegalDriveTypeException;
import frc.robot.subsystems.drive.basicSwerve.config.SwerveModuleConstants;

import java.util.LinkedList;
import java.util.List;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import org.littletonrobotics.junction.Logger;
import java.util.Optional;

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
    private ShooterHopper shooterHopper;
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


            if(Toggles.useVision){
                console("Create vision" + ((Toggles.useDrive && drivetrain != null) ? " (with drive)" : " (without drive)"));
                vision = new VisionSubsystem(drivetrain, "limelight");
            }
            if (Toggles.useIntake) {
                console("Create intake");
                intake = new Intake();
            }

            if (Toggles.useShooter) {
                console("Create shooter");
                shooterHopper = new ShooterHopper();
                if (Toggles.usePidTuner) {
                    shooterTuner = new SingleMotorTuner("Shooter Tuner", shooterHopper.getPidConfig());
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
        
        new Trigger(() -> elasticMotorController.homeButton.get()).onTrue(Commands.runOnce(() -> {
                intake.requestHome();
                elasticMotorController.homeButton.setValue(false);
                System.out.println("is homed?: " + intake.hasBeenHomed);
            }, intake));

        new Trigger(() -> elasticMotorController.goToIdle.get()).onTrue(Commands.runOnce(() -> {
                intake.requestIdle();
                elasticMotorController.homeButton.setValue(false);
            }, intake));
        new Trigger(() -> elasticMotorController.rumble.get()).onTrue(Commands.runOnce(() -> {
            intake.requestRumble();
            elasticMotorController.rumble.setValue(false);
        }, intake));
        new Trigger(() -> elasticMotorController.intakeDown.get()).onTrue(Commands.runOnce(() -> {
        intake.requestIntakeDown();
        elasticMotorController.intakeDown.setValue(false);
        }, intake));
        

        if (shooterHopper != null) {
            Trigger shootTrigger = new Trigger(() -> elasticMotorController.shootButton.get());
            shootTrigger.onTrue(new ShootCommand(shooterHopper));
        }

        if (intake != null) {
            new Trigger(() -> joystick.intakeTogglePressed())
                .onTrue(Commands.runOnce(() -> intake.toggleIntakePosition(), intake));

            new Trigger(() -> joystick.getLeftTriggerAxis() > 0.2)
                .whileTrue(new RunIntake(intake, true));
        }
        console("[Init] configureBindings");

        if (Toggles.useDrive) {
            JoyStickDrive driveWithJoystick = new JoyStickDrive(drivetrain, joystick);

            drivetrain.setDefaultCommand(driveWithJoystick);
            new Trigger(() -> joystick.wheelRadiusCharacterization())
                .onTrue(DriveCommands.wheelRadiusCharacterization(drivetrain));
//            new Trigger(() -> joystick.wheelRadiusCharacterization())
//                .onTrue(DriveCommands.wheelRadiusCharacterizationAll(drivetrain));

            new Trigger(() -> joystick.resetGyro())
                .onTrue(new InstantCommand(() -> drivetrain.resetOrientation()));

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

        if (Toggles.useShooter && shooterHopper != null) {

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
