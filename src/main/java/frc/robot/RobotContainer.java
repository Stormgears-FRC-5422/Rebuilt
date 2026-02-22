// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Toggles;
import frc.robot.commands.AutoCommandFactory;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FaceTowardHub;
import frc.robot.commands.IntakeHome;
import frc.robot.commands.JoyStickDrive;
import frc.robot.commands.RunIntake;
import frc.robot.commands.ShootCommand;
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

public class RobotContainer {
    private final ElasticMotorController elasticMotorController = new ElasticMotorController();
    RobotState robotState;
    // **********
    // Control
    // **********
    RebuiltJoystick joystick;
    @SuppressWarnings("unused")
    private DrivetrainBase drivetrain;
    @SuppressWarnings("unused")
    private Lights lights;
    private VisionSubsystem vision;
    private AutoCommandFactory autoCommandFactory;
    private ShooterHopper shooterHopper;
    private Intake intake;
    //elastic control
    private SingleMotorTuner shooterTuner;

    // **********
    // Commands
    // **********
    // (no pre-created commands)

    public RobotContainer() {
        console("[Init] RobotContainer");
        robotState = RobotState.getInstance();

        try {
            if (Toggles.useDrive) {
                console("Create drive type " + Constants.Drive.driveType);
                drivetrain = DrivetrainFactory.getInstance(Constants.Drive.driveType, Constants.Drive.driveSubtype);
                // FaceTowardHub is created in configureBindings when joystick is available
            }
        } catch (IllegalDriveTypeException e) {
            console("Error: Invalid drive type specified: " + e.getMessage());
        } catch (RuntimeException e) {
            console("Error initializing RobotContainer: " + e.getMessage());
        }

        if (Toggles.useVision) {
            console("Create vision" + ((Toggles.useDrive && drivetrain != null) ? " (with drive)" : " (without drive)"));
            ArrayList<String> limelights = new ArrayList<>();
            for(Field f : Constants.Limelights.class.getFields()){
                try{
                    limelights.add((String) f.get(null));
                } catch (IllegalAccessException e){
                    return;
                }

            }
            vision = new VisionSubsystem(drivetrain, limelights.toArray(new String[0]));
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

        if (drivetrain != null) {
            autoCommandFactory = new AutoCommandFactory(drivetrain, shooterHopper, intake);
        }

        console("[DONE] RobotContainer");
    }

    private static double slider(double x, double low, double high) {
        return Math.max(low, Math.min(high, x));
    }

    public void onJoysticksAvaiable() {
        console("Configuring Joystick and bindings");
        try {
            joystick = RebuiltJoystickFactory.getInstance(Constants.ButtonBoard.driveJoystick, Constants.ButtonBoard.driveJoystickPort);
            configureBindings();
            robotState.setJoystickReady(true);
        } catch (IllegalJoystickTypeException e) {
            console("disabledPeriodic: Error configuring Joystick and button board" + e.getMessage());
        }
    }

    private void configureBindings() {

        if (Toggles.useIntake) {
            new Trigger(() -> elasticMotorController.homeIntake.get())
                .onTrue(new IntakeHome(intake));

            new Trigger(() -> elasticMotorController.goToIdle.get()).onTrue(Commands.runOnce(() -> {
                intake.requestIdle();
                elasticMotorController.homeButton.setValue(false);
            }, intake));

            new Trigger(() -> elasticMotorController.rumble.get()).onTrue(Commands.runOnce(() -> {
                intake.requestRumble();
                elasticMotorController.rumble.setValue(false);
            }, intake));

            new Trigger(() -> elasticMotorController.outtakeIntake.get()).onTrue(Commands.runOnce(() -> {
                intake.setOuttakeRollers(true);
            }, intake));
            new Trigger(() -> elasticMotorController.outtakeIntake.get()).onFalse(Commands.runOnce(() -> {
                intake.setOuttakeRollers(false);
            }, intake));

            new Trigger(() -> elasticMotorController.intakeDown.get()).onTrue(Commands.runOnce(() -> {
                intake.toggleIntakePosition();
            }, intake));
            new Trigger(() -> elasticMotorController.intakeDown.get()).onFalse(Commands.runOnce(() -> {
                intake.toggleIntakePosition();
            }, intake));

            new Trigger(() -> elasticMotorController.outtakeShooter.get()).onTrue(Commands.runOnce(() -> {
                shooterHopper.setState(ShooterHopper.ShooterState.OUTTAKE);
                elasticMotorController.outtakeShooter.setValue(false);
            }, shooterHopper));

            new Trigger(() -> joystick.intakeTogglePressed()).onTrue(Commands.runOnce(() -> intake.toggleIntakePosition(), intake));
            new Trigger(() -> joystick.getIntakeRun()).whileTrue(new RunIntake(intake));
            new Trigger(() -> joystick.shooterIsHeldManual()).whileTrue(new ShootCommand(shooterHopper, false));
            new Trigger(() -> joystick.enableRumble()).onTrue(Commands.runOnce(() -> intake.requestRumble(), intake));
            new Trigger(() -> joystick.turnTowardsHub()).whileTrue(new FaceTowardHub(drivetrain));

        }

        if (shooterHopper != null) {
            Trigger shootTrigger = new Trigger(() -> elasticMotorController.shootButton.get());
            shootTrigger.onTrue(new ShootCommand(shooterHopper, false));
        }
        console("[Init] configureBindings");

        if (Toggles.useDrive) {
            JoyStickDrive driveWithJoystick = new JoyStickDrive(drivetrain, joystick);

            drivetrain.setDefaultCommand(driveWithJoystick);
            new Trigger(() -> joystick.wheelRadiusCharacterization()).onTrue(DriveCommands.wheelRadiusCharacterization(drivetrain));
            new Trigger(() -> joystick.resetGyro()).onTrue(new InstantCommand(() -> drivetrain.resetOrientation()));

//            new Trigger(() -> joystick.wheelRadiusCharacterization())
//                .onTrue(DriveCommands.wheelRadiusCharacterizationAll(drivetrain));

            // Need a test here to allow this based on the drive type.
//            HoldSwerveModuleStates holdCommand = new HoldSwerveModuleStates(drivetrain, MetersPerSecond.of(0.5), Degrees.of(90));
//            new Trigger(() -> joystick.holdSwerveState()).whileTrue(holdCommand);
        }

        console("[DONE] configureBindings");
    }

    public void periodic() {
        if (shooterTuner != null) {
            shooterTuner.periodic();
        }

        if (Toggles.useShooter && shooterHopper != null) {
            ;;;
        }
    }

    /**
     * Updates alliance state from DriverStation/FMS.
     * Respects debug override when Constants.Debug.debug is true and defaultAlliance is set.
     * When alliance becomes available (or changes), calls onAllianceUpdated().
     */
    public void updateAlliance() {
        RobotState.StateAlliance newAlliance = RobotState.StateAlliance.MISSING;
        String defaultAlliance = Constants.Debug.defaultAlliance.toLowerCase();

        if (!Constants.Debug.debug || defaultAlliance.equals("auto")) {
            // Use DriverStation/FMS
            DriverStation.refreshData();
            Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

            if (alliance.isPresent()) {
                newAlliance = alliance.get() == DriverStation.Alliance.Blue ? RobotState.StateAlliance.BLUE : RobotState.StateAlliance.RED;
            }
        } else {
            // Debug override active
            newAlliance = defaultAlliance.equals("blue") ? RobotState.StateAlliance.BLUE : RobotState.StateAlliance.RED;
        }

        RobotState.StateAlliance previousAlliance = robotState.getAlliance();

        // Call onAllianceUpdated when alliance first becomes known or changes
        if (newAlliance != RobotState.StateAlliance.MISSING && (previousAlliance == RobotState.StateAlliance.MISSING || previousAlliance != newAlliance)) {
            robotState.setAlliance(newAlliance);
            onAllianceUpdated();
        }
    }

    /**
     * Resets the drivetrain pose to the initial pose from debug config,
     * remapped for the current alliance.
     */
    public void resetInitialPose() {
        if (!Toggles.useDrive || drivetrain == null) {
            return;
        }

        Pose2d initialPose;
        if (Constants.Debug.debug && !robotState.getAlliance().equals(RobotState.StateAlliance.MISSING)) {
            // Use debug pose settings, remapped for alliance
            Pose2d bluePose = new Pose2d(Constants.Debug.initPoseX, Constants.Debug.initPoseY, Rotation2d.fromDegrees(Constants.Debug.initPoseDegrees));
            initialPose = RebuiltField.remapPose(bluePose, robotState.getAlliance());
        } else {
            // Default pose - AutoChooser should set the pose itself for matches
            initialPose = new Pose2d();
        }
        drivetrain.setPose(initialPose);
    }

    public void updateAutoPose() {
        if (autoCommandFactory == null) return;
        autoCommandFactory.getInitialPose().ifPresent(drivetrain::setPose);
    }

    /**
     * Called when alliance is first set or changes.
     * Resets initial pose and performs any other alliance-dependent initialization.
     */
    private void onAllianceUpdated() {
        console("Alliance updated to " + RobotState.getInstance().getAlliance());
        if (autoCommandFactory != null) {
            autoCommandFactory.initialize();
        }
        resetInitialPose();
    }

    private void console(String message) {
        System.out.println("RobotContainer : " + message);
    }

    public Command getAutonomousCommand() {
        if (autoCommandFactory == null) return null;
        return autoCommandFactory.getChosenCommand();
    }
}
