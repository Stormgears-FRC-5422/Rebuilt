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
import edu.wpi.first.wpilibj.Timer;
import frc.robot.elastic.GameDataDashboard;
import frc.robot.game.GameDataTracker;
import frc.robot.game.RebuiltField;
import frc.utils.Alert;
import frc.utils.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Toggles;
import frc.robot.commands.AlignNarrow;
import frc.robot.commands.AutoCommandFactory;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.FunnelToAllianceZone;
import frc.robot.commands.JoyStickDrive;
import frc.robot.commands.MoveWhileShoot;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.diagnostic.IntakePositionDiagnostic;
import frc.robot.commands.diagnostic.IntakeRollerDiagnostic;
import frc.robot.commands.diagnostic.ShooterDiagnostic;
import frc.robot.commands.diagnostic.SystemCheck;
import frc.robot.commands.climber.ClimberHome;
import frc.robot.commands.climber.ClimberSequence;
import frc.robot.commands.intake.DynamicRumble;
import frc.robot.commands.intake.IntakeGoToPosition;
import frc.robot.commands.intake.IntakeHome;
import frc.robot.commands.intake.RunRoller;
import frc.robot.commands.intake.SimpleRumble;
import frc.robot.commands.intake.IntakeManual;
import frc.robot.Options.ShooterOptions;
import frc.robot.elastic.ElasticMotorController;
import frc.robot.elastic.SingleMotorTuner;
import frc.robot.joysticks.IllegalJoystickTypeException;
import frc.robot.joysticks.RebuiltJoystick;
import frc.robot.joysticks.RebuiltJoystickFactory;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.ShooterHopper;
import frc.robot.subsystems.EstimationManager;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.drive.DrivetrainFactory;
import frc.robot.subsystems.drive.IllegalDriveTypeException;
import frc.robot.subsystems.misc.SensorSubsystem;

public class RobotContainer {
    private final ElasticMotorController elasticMotorController = new ElasticMotorController();
    private final Alert autoWinnerNotSet = new Alert("!!! AUTO WINNER NOT SET !!!", AlertType.ERROR);
    private final Timer teleopElapsedTimer = new Timer();
    private final GameDataDashboard gameDataDashboard = new GameDataDashboard();
    RobotState robotState;
    // **********
    // Control
    // **********
    RebuiltJoystick firstJoystick;
    RebuiltJoystick secondJoystick;
    @SuppressWarnings("unused")
    private DrivetrainBase drivetrain;
    @SuppressWarnings("unused")
    private Lights lights;
    @SuppressWarnings("unused")
    private VisionSubsystem vision;
    private EstimationManager estimationManager;
    private AutoCommandFactory autoCommandFactory;
    private ShooterHopper shooterHopper;
    @SuppressWarnings("unused")
    private SensorSubsystem sensorSubsystem;
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
            throw new RuntimeException("Invalid drive type specified: " + e.getMessage(), e);
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

        if (Toggles.useDrive) {
            estimationManager = new EstimationManager(drivetrain, vision);
        }

        if (Toggles.useIntake) {
            console("Create intake");
            intake = new Intake();
            if (Toggles.usePidTuner) {
                intakeTuner = new SingleMotorTuner("IntakeArmPID", intake.getPidConfig())
                    .withGravity();
            }
        }

        if (Toggles.useClimber) {
            console("Create climber");
            climber = new Climber();
        }

        if (Toggles.useSensorSubsystem) {
            console("Create sensor subsystem");
            sensorSubsystem = new SensorSubsystem();
        }

        if (Toggles.useShooter) {
            console("Create shooter");
            shooterHopper = new ShooterHopper();
            if (Toggles.usePidTuner) {
                shooterTuner = new SingleMotorTuner("ShooterPID", shooterHopper.getPidConfig())
                    .withOutputLimits();
            }
        }

        if (Toggles.useLights) {
            lights = new Lights();
        }

        if (drivetrain != null) {
            autoCommandFactory = new AutoCommandFactory(drivetrain, shooterHopper, intake);
            if (shooterHopper != null) {
                drivetrain.setShootingSupplier(shooterHopper::isShooting);
            }
        }

        console("[DONE] RobotContainer");
    }

    public void onJoysticksAvaiable() {
        console("Configuring Joystick and bindings");
        try {
            firstJoystick = RebuiltJoystickFactory.getInstance(Constants.ButtonBoard.driveJoystick, Constants.ButtonBoard.driveJoystickPort);
            if (!tryConnectSecondJoystick()) {
                console("Second joystick not connected on port " + Constants.ButtonBoard.driveJoystickPort2);
            }
            configureBindings();
            robotState.setJoystickReady(true);
        } catch (IllegalJoystickTypeException e) {
            console("disabledPeriodic: Error configuring Joystick and button board" + e.getMessage());
        }
    }

    /** Attempts to connect the second joystick if not already connected. Returns true if connected. */
    public boolean tryConnectSecondJoystick() {
        if (secondJoystick != null) return true;
        if (DriverStation.isJoystickConnected(Constants.ButtonBoard.driveJoystickPort2)) {
            try {
                secondJoystick = RebuiltJoystickFactory.getInstance(Constants.ButtonBoard.driveJoystick2, Constants.ButtonBoard.driveJoystickPort2);
                configureSecondJoystickBindings();
                return true;
            } catch (IllegalJoystickTypeException e) {
                return false;
            }
        }
        return false;
    }

    private void configureBindings() {

        if (Toggles.useDrive) {
            new Trigger(() -> elasticMotorController.alignNarrow.get())
                .whileTrue(new AlignNarrow(drivetrain, firstJoystick));
        }

        if (Toggles.useIntake) {
            new Trigger(() -> elasticMotorController.homeIntake.get())
                .whileTrue(new IntakeHome(intake)
                    .finallyDo(() -> elasticMotorController.homeIntake.setValue(false)));

            new Trigger(() -> firstJoystick.getIntakeRun()).whileTrue(new RunRoller(intake, true));

            new Trigger(() -> firstJoystick.enableRumble()).onTrue(Commands.either(
                new DynamicRumble(intake),
                new SimpleRumble(intake),
                () -> Constants.Intake.useDynamicRumble));

            new Trigger(() -> firstJoystick.intakeTogglePressed())
                .onTrue(new IntakeGoToPosition(intake, Constants.Intake.intakingAngle));

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
        }

        if (Toggles.useDrive && Toggles.useShooter && Toggles.useIntake) {
            new Trigger(() -> firstJoystick.autoShoot()).whileTrue(
                new ParallelCommandGroup(
                    new ShootCommand(shooterHopper, true),
                    new MoveWhileShoot(drivetrain, firstJoystick, shooterHopper)
                )
            );
        } else if (Toggles.useShooter && Toggles.useIntake) {
            new Trigger(() -> firstJoystick.autoShoot()).whileTrue(new ShootCommand(shooterHopper, true));
        }

        if(Toggles.useShooter && Toggles.useIntake){
            new Trigger(() -> firstJoystick.manualShoot()).whileTrue(
               new ShootCommand(shooterHopper, true));
        }

        if (shooterHopper != null) {
            new Trigger(() -> elasticMotorController.shootButton.get())
                .whileTrue(new ShootCommand(shooterHopper, true)
                    .finallyDo(() -> elasticMotorController.shootButton.setValue(false)));

            // SysId characterization toggles (dashboard options)
            ShooterOptions sysIdOpts = ShooterOptions.create();
            new Trigger(() -> sysIdOpts.sysIdQuasistaticFwd.get())
                .whileTrue(shooterHopper.sysIdQuasistatic(SysIdRoutine.Direction.kForward, sysIdOpts.sysIdQuasistaticFwd));
            new Trigger(() -> sysIdOpts.sysIdQuasistaticRev.get())
                .whileTrue(shooterHopper.sysIdQuasistatic(SysIdRoutine.Direction.kReverse, sysIdOpts.sysIdQuasistaticRev));
            new Trigger(() -> sysIdOpts.sysIdDynamicFwd.get())
                .whileTrue(shooterHopper.sysIdDynamic(SysIdRoutine.Direction.kForward, sysIdOpts.sysIdDynamicFwd));
            new Trigger(() -> sysIdOpts.sysIdDynamicRev.get())
                .whileTrue(shooterHopper.sysIdDynamic(SysIdRoutine.Direction.kReverse, sysIdOpts.sysIdDynamicRev));
        }

        // --- Diagnostic triggers ---
        if (shooterHopper != null) {
            new Trigger(() -> elasticMotorController.diagShooter.get() || elasticMotorController.diagHopper.get())
                .whileTrue(new ShooterDiagnostic(shooterHopper,
                        elasticMotorController.diagShooter::get,
                        elasticMotorController.diagHopper::get)
                    .finallyDo(() -> {
                        elasticMotorController.diagShooter.setValue(false);
                        elasticMotorController.diagHopper.setValue(false);
                    }));
        }

        if (intake != null) {
            new Trigger(() -> elasticMotorController.diagIntakeRoller.get())
                .whileTrue(new IntakeRollerDiagnostic(intake)
                    .finallyDo(() -> elasticMotorController.diagIntakeRoller.setValue(false)));

            new Trigger(() -> elasticMotorController.diagIntakeUpDown.get())
                .whileTrue(new IntakePositionDiagnostic(intake)
                    .finallyDo(() -> elasticMotorController.diagIntakeUpDown.setValue(false)));
        }

        if (shooterHopper != null && intake != null) {
            new Trigger(() -> elasticMotorController.systemCheck.get())
                .whileTrue(new SystemCheck(shooterHopper, intake)
                    .finallyDo(() -> elasticMotorController.systemCheck.setValue(false)));
        }

        // --- Game data overrides ---
        GameDataTracker.setAllianceWinOverride(
            () -> {
                if (secondJoystick == null) return Optional.empty();
                if (secondJoystick.lostAutoOverride()) return Optional.of(false);
                if (secondJoystick.wonAutoOverride())  return Optional.of(true);
                return Optional.empty();
            });

        // Alert + rumble when game data is missing after 1s in teleop
        new Trigger(DriverStation::isTeleopEnabled)
            .onTrue(Commands.runOnce(teleopElapsedTimer::restart).withName("Restart Teleop Timer"));

        new Trigger(DriverStation::isTeleopEnabled)
            .and(() -> !GameDataTracker.isGameDataReceived())
            .and(() -> GameDataTracker.getAllianceWinOverride().isEmpty())
            .and(() -> teleopElapsedTimer.hasElapsed(1.0))
            .whileTrue(Commands.runEnd(
                () -> {
                    if (Constants.ButtonBoard.rumbleOnAlert) {
                        firstJoystick.setRumble(1.0);
                        if (secondJoystick != null) secondJoystick.setRumble(1.0);
                    }
                    autoWinnerNotSet.set(true);
                },
                () -> {
                    firstJoystick.setRumble(0.0);
                    if (secondJoystick != null) secondJoystick.setRumble(0.0);
                    autoWinnerNotSet.set(false);
                }));

        console("[Init] configureBindings");

        if (Toggles.useDrive) {
            JoyStickDrive driveWithFirstJoystick = new JoyStickDrive(drivetrain, firstJoystick);

            drivetrain.setDefaultCommand(driveWithFirstJoystick);
            // new Trigger(() -> firstJoystick.wheelRadiusCharacterization()).onTrue(DriveCommands.wheelRadiusCharacterization(drivetrain));
            new Trigger(() -> firstJoystick.resetGyro()).onTrue(new InstantCommand(() -> drivetrain.resetOrientation()));
            new Trigger(() -> ShooterOptions.create().moveWhileShoot.get()).whileTrue(new MoveWhileShoot(drivetrain, firstJoystick, shooterHopper));

//            new Trigger(() -> firstJoystick.wheelRadiusCharacterization())
//                .onTrue(DriveCommands.wheelRadiusCharacterizationAll(drivetrain));

        }

    }

    private void configureSecondJoystickBindings() {
        console("Configuring second joystick bindings");

        if (Toggles.useDrive) {
            new Trigger(() -> secondJoystick.safe())
                .onTrue(new IntakeGoToPosition(intake, Constants.Intake.cruiseAngleDegrees));
        }

        if (Toggles.useIntake) {
            intake.setDefaultCommand(new IntakeManual(intake, secondJoystick));
            new Trigger(() -> secondJoystick.intakeMoved())
                .onTrue(Commands.runOnce(() -> intake.getCurrentCommand().cancel())
                    .onlyIf(() -> intake.getCurrentCommand() != intake.getDefaultCommand()));

            new Trigger(() -> secondJoystick.intakeHome())
                .onTrue(new IntakeHome(intake));

            new Trigger(() -> secondJoystick.intakeTogglePressed())
                .onTrue(new IntakeGoToPosition(intake, Constants.Intake.intakingAngle));
        }

        if (Toggles.useClimber) {
            new Trigger(() -> secondJoystick.climbHome()).onTrue(new ClimberHome(climber));
            new Trigger(() -> secondJoystick.climbSequence()).onTrue(new ClimberSequence(climber));
            new Trigger(() -> secondJoystick.climbManualUp()).whileTrue(
                Commands.startEnd(() -> climber.setManualDirection(Climber.Direction.FORWARD),
                                  () -> climber.setManualDirection(null), climber));
            new Trigger(() -> secondJoystick.climbManualDown()).whileTrue(
                Commands.startEnd(() -> climber.setManualDirection(Climber.Direction.BACKWARD),
                                  () -> climber.setManualDirection(null), climber));
        }

        if (Toggles.useDrive && Toggles.useShooter && Toggles.useIntake) {
            new Trigger(() -> secondJoystick.shootToAlliance()).whileTrue(
                new ParallelCommandGroup(
                    new ShootCommand(shooterHopper, false),
                    new FunnelToAllianceZone(drivetrain, secondJoystick, shooterHopper)
                )
            );

            new Trigger(() -> secondJoystick.autoShoot()).whileTrue(new ShootCommand(shooterHopper, true));
        } else if (Toggles.useShooter && Toggles.useIntake) {
            new Trigger(() -> secondJoystick.autoShoot()).whileTrue(new ShootCommand(shooterHopper, true));
        }

        if (Toggles.useShooter) {
            new Trigger(() -> firstJoystick.testAutoShoot())
                .whileTrue(new AutoShootCommand(shooterHopper, true));
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

        gameDataDashboard.periodic();
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
