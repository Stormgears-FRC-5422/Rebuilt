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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Toggles;
import frc.robot.commands.AutoCommandFactory;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FunnelToAllianceZone;
import frc.robot.commands.JoyStickDrive;
import frc.robot.commands.MoveWhileShoot;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.intake.DynamicRumble;
import frc.robot.commands.intake.RunRoller;
import frc.robot.commands.intake.IntakeDiagnostic;
import frc.robot.commands.intake.IntakeGoToPosition;
import frc.robot.commands.intake.IntakeHome;
import frc.robot.commands.intake.SimpleRumble;
import frc.robot.elastic.ElasticMotorController;
import frc.robot.elastic.SingleMotorTuner;
import frc.robot.joysticks.IllegalJoystickTypeException;
import frc.robot.joysticks.RebuiltJoystick;
import frc.robot.joysticks.RebuiltJoystickFactory;
import frc.robot.commands.climber.ClimberHome;
import frc.robot.commands.climber.ClimberSequence;
import frc.robot.subsystems.Climber;
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
    RebuiltJoystick joystick2;
    @SuppressWarnings("unused")
    private DrivetrainBase drivetrain;
    @SuppressWarnings("unused")
    private Lights lights;
    private VisionSubsystem vision;
    private AutoCommandFactory autoCommandFactory;
    private ShooterHopper shooterHopper;
    private Intake intake;
    private Climber climber;
    //elastic control
    private SingleMotorTuner shooterTuner;
    private SingleMotorTuner intakeTuner;

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
            if (Toggles.usePidTuner) {
                intakeTuner = new SingleMotorTuner("Intake Tuner", intake.getPidConfig());
            }
        }

        if (Toggles.useClimber) {
            console("Create climber");
            climber = new Climber();
        }

        if (Toggles.useShooter) {
            console("Create shooter");
            shooterHopper = new ShooterHopper(elasticMotorController);
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
            joystick2 = RebuiltJoystickFactory.getInstance(Constants.ButtonBoard.driveJoystick2, Constants.ButtonBoard.driveJoystickPort2);
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
                
            new Trigger(() -> joystick.getIntakeRun()).whileTrue(new RunRoller(intake, true));


            new Trigger(() -> joystick.enableRumble()).onTrue(Commands.either(
                new DynamicRumble(intake),
                new SimpleRumble(intake),
                () -> frc.robot.Constants.Intake.useDynamicRumble));

            new Trigger(() -> joystick.shootToAlliance()).whileTrue(
                new ParallelCommandGroup(
                    new FunnelToAllianceZone(drivetrain, joystick, shooterHopper),
                    new ShootCommand(shooterHopper, false)
                )
            );
        }

        if (Toggles.useClimber) {
            new Trigger(() -> elasticMotorController.homeClimber.get())
                .onTrue(new ClimberHome(climber));
            new Trigger(() -> elasticMotorController.climberSequence.get())
                .onTrue(new ClimberSequence(climber));
            new Trigger(() -> elasticMotorController.climberManualUp.get()).whileTrue(
                Commands.startEnd(() -> climber.setManualDirection(Climber.Direction.FORWARD),
                                  () -> climber.setManualDirection(null), climber));
            new Trigger(() -> elasticMotorController.climberManualDown.get()).whileTrue(
                Commands.startEnd(() -> climber.setManualDirection(Climber.Direction.BACKWARD),
                                  () -> climber.setManualDirection(null), climber));

            new Trigger(() -> joystick2.climbHome()).onTrue(new ClimberHome(climber));
            new Trigger(() -> joystick2.climbSequence()).onTrue(new ClimberSequence(climber));
            new Trigger(() -> joystick2.climbManualUp()).whileTrue(
                Commands.startEnd(() -> climber.setManualDirection(Climber.Direction.FORWARD),
                                  () -> climber.setManualDirection(null), climber));
            new Trigger(() -> joystick2.climbManualDown()).whileTrue(
                Commands.startEnd(() -> climber.setManualDirection(Climber.Direction.BACKWARD),
                                  () -> climber.setManualDirection(null), climber));
        }

        if (Toggles.useDrive && Toggles.useShooter) {
            new Trigger(() -> joystick.autoShoot()).whileTrue(
                new ParallelCommandGroup(
                    new ShootCommand(shooterHopper, true),
                    new MoveWhileShoot(drivetrain, joystick, shooterHopper),
                    Commands.either(new DynamicRumble(intake), Commands.startEnd(
                        intake::requestRumble, intake::requestIdle, intake),
                    () -> frc.robot.Constants.Intake.useDynamicRumble)
                )
            );

            new Trigger(() -> joystick2.shootToAlliance()).whileTrue(
                new ParallelCommandGroup(
                    new ShootCommand(shooterHopper, false),
                    new FunnelToAllianceZone(drivetrain, joystick, shooterHopper),
                    Commands.either(new DynamicRumble(intake), Commands.startEnd(
                            intake::requestRumble, intake::requestIdle, intake),
                        () -> frc.robot.Constants.Intake.useDynamicRumble)
                )
            );
        } else if (Toggles.useShooter && Toggles.useIntake) {
            new Trigger(() -> joystick.autoShoot()).whileTrue(
                new ParallelCommandGroup(new ShootCommand(shooterHopper, true),
                    Commands.either(new DynamicRumble(intake), Commands.startEnd(
                            intake::requestRumble, intake::requestIdle, intake),
                        () -> frc.robot.Constants.Intake.useDynamicRumble)));

            new Trigger(() -> joystick2.autoShoot()).whileTrue(
                new ParallelCommandGroup(new ShootCommand(shooterHopper, true),
                    Commands.either(new DynamicRumble(intake), Commands.startEnd(
                            intake::requestRumble, intake::requestIdle, intake),
                        () -> frc.robot.Constants.Intake.useDynamicRumble)));
        }

        if(Toggles.useShooter && Toggles.useIntake){
            new Trigger(() -> joystick.manualShoot()).whileTrue(
                new ParallelCommandGroup(new ShootCommand(shooterHopper, true),
                    Commands.either(new DynamicRumble(intake), Commands.startEnd(
                            intake::requestRumble, intake::requestIdle, intake),
                        () -> frc.robot.Constants.Intake.useDynamicRumble)));
        }

        if (shooterHopper != null) {
            Trigger shootTrigger = new Trigger(() -> elasticMotorController.shootButton.get());
            shootTrigger.onTrue(new ShootCommand(shooterHopper, true));
        }
        console("[Init] configureBindings");

        if (Toggles.useDrive) {
            JoyStickDrive driveWithJoystick = new JoyStickDrive(drivetrain, joystick);

            drivetrain.setDefaultCommand(driveWithJoystick);
            // new Trigger(() -> joystick.wheelRadiusCharacterization()).onTrue(DriveCommands.wheelRadiusCharacterization(drivetrain));
            new Trigger(() -> joystick.resetGyro()).onTrue(new InstantCommand(() -> drivetrain.resetOrientation()));
            new Trigger(() -> elasticMotorController.moveWhileShoot.get()).whileTrue(new MoveWhileShoot(drivetrain, joystick, shooterHopper));

//            new Trigger(() -> joystick.wheelRadiusCharacterization())
//                .onTrue(DriveCommands.wheelRadiusCharacterizationAll(drivetrain));

        }

    }

    public void periodic() {
        if (shooterTuner != null) {
            shooterTuner.periodic();
        }
        if (intakeTuner != null) {
            intakeTuner.periodic();
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