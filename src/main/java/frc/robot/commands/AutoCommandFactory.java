package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

import frc.robot.commands.diagnostic.IntakePositionDiagnostic;
import frc.robot.commands.intake.*;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import choreo.auto.AutoFactory;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Shooter;
import frc.robot.RobotState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakePosition;
import frc.robot.subsystems.ShooterHopper;
import frc.robot.subsystems.drive.DrivetrainBase;

public class AutoCommandFactory {
    private final DrivetrainBase drivetrainBase;
    private final ShooterHopper shooterHopper;
    private final Intake intake;
    private final RobotState robotState;

    private AutoFactory choreoFactory;
    private final LoggedDashboardChooser<String> autoChooser = new LoggedDashboardChooser<>("Auto Routine");
    private final LoggedDashboardChooser<String> startPositionChooser = new LoggedDashboardChooser<>("Start Position");
    private final LoggedDashboardChooser<String> firstCycleChooser = new LoggedDashboardChooser<>("First Cycle");
    private final LoggedDashboardChooser<String> secondCycleChooser = new LoggedDashboardChooser<>("Second Cycle");

    /** Returns the configured auto-shoot command with safety timeout. */
    private Command shootCmd() {
        return Shooter.useAutoShoot
            ? new AutoShootCommand(shooterHopper, true).withTimeout(7)
            : new TimedShootCommand(shooterHopper, true, 7);
    }

    @AutoLogOutput
    private boolean isDirty;
    private boolean chooserInitialized = false;
    private Command chosenCommand;
    @SuppressWarnings("OptionalUsedAsFieldOrParameterType")
    private Optional<? extends Trajectory<?>> chosenTrajectory = Optional.empty();
    private String chooserTrajectoryName = "CycleAuto";
    private String lastStartPosition = "";
    private String lastFirstCycle = "";
    private String lastSecondCycle = "";

    public AutoCommandFactory(DrivetrainBase drivetrainBase, ShooterHopper shooterHopper, Intake intake) {
        this.robotState = RobotState.getInstance();
        this.drivetrainBase = drivetrainBase;
        this.shooterHopper = shooterHopper;
        this.intake = intake;
    }


    public void initialize() {
        choreoFactory = new AutoFactory(
            drivetrainBase::getPose,
            drivetrainBase::setPose,
            drivetrainBase::followTrajectory,
            robotState.isAllianceRed(),
            drivetrainBase
        );

        loadTrajectories();

        if (!chooserInitialized) {
            autoChooser.addDefaultOption("Cycle Auto", "CycleAuto");
            autoChooser.addOption("Nothing", null);

            startPositionChooser.addDefaultOption("Left", "left");
            startPositionChooser.addOption("Middle", "mid");
            startPositionChooser.addOption("Right", "right");

            firstCycleChooser.addDefaultOption("Mid", "mid");
            firstCycleChooser.addOption("Mid TR", "mid_tr");
//            firstCycleChooser.addOption("Mid TR Steal", "mid_tr_steal"); // TODO: re-enable when tr_steal paths are loaded
//            firstCycleChooser.addOption("Mid Windham", "mid_windham"); // TODO: re-enable when windham paths are loaded
//            firstCycleChooser.addOption("Mid Windham Steal", "mid_windham_steal"); // TODO: re-enable when windham_steal paths are loaded
            firstCycleChooser.addOption("Depot", "depot");
            firstCycleChooser.addOption("Outpost", "outpost");
            firstCycleChooser.addOption("Shoot Only", "shoot_only");

            secondCycleChooser.addDefaultOption("None", "none");
            secondCycleChooser.addOption("Mid", "mid");
            secondCycleChooser.addOption("Mid TR", "mid_tr");
            secondCycleChooser.addOption("Mid TR 2", "mid_tr_2");
//            secondCycleChooser.addOption("Mid Windham", "mid_windham"); // TODO: re-enable when windham paths are loaded
//            secondCycleChooser.addOption("Depot", "depot"); // TODO: re-enable when depot paths are loaded
//            secondCycleChooser.addOption("Double Swipe", "double_swipe"); // TODO: re-enable when double_swipe paths are loaded

            chooserInitialized = true;
        }

        setDirty();
    }


    public Command getChosenCommand() {
        refreshIfDirty();
        return chosenCommand;
    }

    public Optional<Pose2d> getInitialPose() {
        refreshIfDirty();
        return chosenTrajectory
            .flatMap(t -> t.getInitialPose(robotState.isAllianceRed()));
    }

    private void setDirty() {
        this.isDirty = true;
    }

    private void refreshIfDirty() {
        if (choreoFactory == null) return;

        String currentSelection = autoChooser.get();
        String currentStart = startPositionChooser.get();
        String currentFirst = firstCycleChooser.get();
        String currentSecond = secondCycleChooser.get();

        // Always log validity for Elastic dashboard
        if ("CycleAuto".equals(currentSelection)) {
            boolean validCombo = isValidCycleCombo(currentStart, currentFirst, currentSecond);
            Logger.recordOutput("Auto/ValidCombo", validCombo);
            Logger.recordOutput("Auto/CycleDescription", currentStart + " / " + currentFirst + " / " + currentSecond);
        } else {
            Logger.recordOutput("Auto/ValidCombo", currentSelection != null);
            Logger.recordOutput("Auto/CycleDescription", currentSelection != null ? currentSelection : "Nothing");
        }

        boolean selectionChanged = !Objects.equals(currentSelection, chooserTrajectoryName)
            || !Objects.equals(currentStart, lastStartPosition)
            || !Objects.equals(currentFirst, lastFirstCycle)
            || !Objects.equals(currentSecond, lastSecondCycle);

        if (!isDirty && !selectionChanged) return;

        chooserTrajectoryName = currentSelection;
        lastStartPosition = currentStart;
        lastFirstCycle = currentFirst;
        lastSecondCycle = currentSecond;

        if ("CycleAuto".equals(chooserTrajectoryName)) {
            chosenCommand = buildCycleAuto(currentStart, currentFirst, currentSecond);
            chosenTrajectory = loadTrajectory(resolveInitialPoseTrajectory(currentStart, currentFirst));
        } else {
            chosenCommand = Commands.none();
            chosenTrajectory = Optional.empty();
        }

        isDirty = false;
    }

    private void loadTrajectories() {
        // Shoot-only trajectories
        loadTrajectory("left_shoot_only");
        loadTrajectory("mid_shoot_only");
        loadTrajectory("right_shoot_only");

        // Left-side cycle trajectories
        loadTrajectory("left_first_mid");
        loadTrajectory("left_first_mid_tr");
//        loadTrajectory("left_first_mid_tr_steal"); // TODO: re-enable
//        loadTrajectory("left_first_mid_windham"); // TODO: re-enable
//        loadTrajectory("left_first_mid_windham_steal"); // TODO: re-enable
        loadTrajectory("left_first_depot");
        loadTrajectory("left_second_mid");
        loadTrajectory("left_second_mid_tr");
        loadTrajectory("left_second_mid_tr_2");
//        loadTrajectory("left_second_mid_windham"); // TODO: re-enable
//        loadTrajectory("left_second_depot"); // TODO: re-enable
//        loadTrajectory("left_second_double_swipe"); // TODO: re-enable

        // Right-side cycle trajectories
        loadTrajectory("right_first_mid");
        loadTrajectory("right_first_mid_tr");
//        loadTrajectory("right_first_mid_tr_steal"); // TODO: re-enable
//        loadTrajectory("right_first_mid_windham"); // TODO: re-enable
//        loadTrajectory("right_first_mid_windham_steal"); // TODO: re-enable
        loadTrajectory("right_second_mid");
        loadTrajectory("right_second_mid_tr");
//        loadTrajectory("right_second_mid_tr_2"); // TODO: re-enable
//        loadTrajectory("right_second_mid_windham"); // TODO: re-enable
//        loadTrajectory("right_second_double_swipe"); // TODO: re-enable

        // Middle-start cycle trajectories
        loadTrajectory("mid_first_depot");
        loadTrajectory("mid_first_outpost");
    }

    private Optional<? extends Trajectory<?>> loadTrajectory(String trajectoryName) {
        Optional<? extends Trajectory<?>> trajectory =  choreoFactory.cache().loadTrajectory(trajectoryName);
        if (trajectory.isEmpty()){
            System.err.println(trajectoryName + " is not found");
        }
        return trajectory;
    }


    /** Starts the shooter spinning up without requiring the shooter subsystem. */
    private Command spinUp() {
        return Commands.runOnce(() -> {
            shooterHopper.setToHub(true);
            shooterHopper.setUseRealDist(true);
            shooterHopper.setState(ShooterHopper.ShooterState.SPEEDING_UP);
        });
    }

    private Command stopDrive() {
        return Commands.runOnce(() -> drivetrainBase.drive(new ChassisSpeeds(), false), drivetrainBase);
    }

    private Command buildCycleAuto(String start, String firstCycle, String secondCycle) {
        String label = "Cycle Auto: " + start + " / " + firstCycle + " / " + secondCycle;

        // Validate combination
        if (!isValidCycleCombo(start, firstCycle, secondCycle)) {
            System.err.println("Invalid cycle auto combo: " + label);
            return Commands.sequence(new PrintCommand("Invalid: " + label), Commands.none());
        }

        // Shoot only — just drive to scoring position and shoot
        if ("shoot_only".equals(firstCycle)) {
            return shootOnlyAuto(label, start + "_shoot_only");
        }

        // Resolve trajectory names
        String firstTrajectory = start + "_first_" + firstCycle;
        String effectiveSide = resolveEffectiveSide(start, firstCycle);
        String secondTrajectory = resolveSecondTrajectory(effectiveSide, secondCycle);

        List<Command> commands = new ArrayList<>();
        commands.add(new PrintCommand(label));

        if (intake == null || shooterHopper == null) {
            // Diagnostic: just run trajectories without intake/shooter
            commands.add(choreoFactory.trajectoryCmd(firstTrajectory));
            commands.add(stopDrive());
            if (secondTrajectory != null) {
                commands.add(choreoFactory.trajectoryCmd(secondTrajectory));
                commands.add(stopDrive());
            }
            return Commands.sequence(commands.toArray(new Command[0]));
        }

        // First cycle
        commands.add(
            new IntakeHome(intake)
        );
        commands.add(
            Commands.race(
                new IntakeGoToPosition(intake, IntakePosition.BOTTOM),
                new RunRoller(intake, true),
                Commands.deadline(
                    choreoFactory.trajectoryCmd(firstTrajectory),
                    Commands.sequence(Commands.waitSeconds(2.0), spinUp())
                )
            )
        );
        commands.add(stopDrive());
        commands.add(
            Commands.deadline(
                shootCmd(),
                new FaceTowardHub(drivetrainBase),
                new DynamicRumble(intake).andThen(new DynamicRumble(intake))
            )
        );

        // Second cycle (if selected)
        if (secondTrajectory != null) {
            commands.add(Commands.race(
                Commands.sequence(
                    new IntakeGoToPosition(intake, IntakePosition.BOTTOM).withTimeout(1.5),
                    new RunRoller(intake, true)
                ),
                Commands.deadline(
                    choreoFactory.trajectoryCmd(secondTrajectory),
                    Commands.sequence(Commands.waitSeconds(2.0), spinUp())
                )
            ));
            commands.add(stopDrive());
            commands.add(Commands.race(
                shootCmd(),
                new DynamicRumble(intake)
            ));
        }

        return Commands.sequence(commands.toArray(new Command[0]));
    }

    private Command shootOnlyAuto(String label, String trajectoryName) {
        Command trajectory = choreoFactory.trajectoryCmd(trajectoryName);
        Command printName = new PrintCommand(label);

        if (intake == null || shooterHopper == null) return Commands.sequence(printName, trajectory, stopDrive());

        return Commands.sequence(
            printName,
            Commands.parallel(
                new IntakeHome(intake),
                trajectory
            ),
            stopDrive(),
            Commands.parallel(
                shootCmd(),
                new DynamicRumble(intake)
            )
        );
    }

    private String resolveEffectiveSide(String start, String firstCycle) {
        if ("left".equals(start) || "right".equals(start)) return start;
        // Middle start: depot and mid end at left scoring position
        return "left";
    }

    private String resolveSecondTrajectory(String effectiveSide, String secondCycle) {
        if ("none".equals(secondCycle)) return null;
        return effectiveSide + "_second_" + secondCycle;
    }

    private String resolveInitialPoseTrajectory(String start, String firstCycle) {
        if ("shoot_only".equals(firstCycle)) return start + "_shoot_only";
        return start + "_first_" + firstCycle;
    }


    private boolean isValidCycleCombo(String start, String firstCycle, String secondCycle) {
        if ("mid".equals(start) && firstCycle.startsWith("mid")) return false;
        if ("right".equals(start) && ("depot".equals(firstCycle) || "depot".equals(secondCycle))) return false;
        if ("outpost".equals(firstCycle) && !"mid".equals(start)) return false;
        return true;
    }
}
