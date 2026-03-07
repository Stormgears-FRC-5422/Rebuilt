package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

import edu.wpi.first.wpilibj2.command.WaitCommand;
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
import frc.robot.Constants.Auto;
import frc.robot.commands.intake.DynamicRumble;
import frc.robot.commands.intake.IntakeGoToPosition;
import frc.robot.commands.intake.IntakeHome;
import frc.robot.commands.intake.RunRoller;
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

    @AutoLogOutput
    private boolean isDirty;
    private boolean chooserInitialized = false;
    private Command chosenCommand;
    @SuppressWarnings("OptionalUsedAsFieldOrParameterType")
    private Optional<? extends Trajectory<?>> chosenTrajectory = Optional.empty();
    private String chooserTrajectoryName = Auto.path;
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
            // Main auto chooser — legacy autos + cycle auto
            autoChooser.addDefaultOption(Auto.path, Auto.path);
            autoChooser.addOption("Nothing", null);
            autoChooser.addOption("Cycle Auto", "CycleAuto");
            autoChooser.addOption("Straight", "Straight");
            autoChooser.addOption("Reload", "Reload");
            autoChooser.addOption("Sweeper", "Sweeper");
            autoChooser.addOption("TwoCycleLeft", "TwoCycleLeft");
            autoChooser.addOption("TwoCycleRight", "TwoCycleRight");
            autoChooser.addOption("CircularRabbit", "CircularRabbit");
            autoChooser.addOption("HokeyPokeyRabbit", "HokeyPokeyRabbit");
            autoChooser.addOption("PoopingSloth", "PoopingSloth");
            autoChooser.addOption("Tortoise", "Tortoise");
            autoChooser.addOption("Left Close Outpost", "left_close_outpost");
            autoChooser.addOption("Right Close Outpost", "right_close_outpost");

            // Cycle auto sub-choosers
            startPositionChooser.addDefaultOption("Left", "left");
            startPositionChooser.addOption("Middle", "mid");
            startPositionChooser.addOption("Right", "right");

            firstCycleChooser.addDefaultOption("Mid", "mid");
            firstCycleChooser.addOption("Depot", "depot");
            firstCycleChooser.addOption("Outpost", "outpost");
            firstCycleChooser.addOption("Shoot Only", "shoot_only");
            firstCycleChooser.addOption("Close Outpost", "close_outpost");

            secondCycleChooser.addDefaultOption("None", "none");
            secondCycleChooser.addOption("Mid", "mid");
            secondCycleChooser.addOption("Depot", "depot");
            secondCycleChooser.addOption("Outpost", "outpost");
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
            chosenCommand = buildCommand(chooserTrajectoryName);
            chosenTrajectory = (chooserTrajectoryName != null)
                ? loadTrajectory(chooserTrajectoryName)
                : Optional.empty();
        }

        isDirty = false;
    }

    private void loadTrajectories() {
        // Legacy trajectories
        loadTrajectory("Straight");
        loadTrajectory("Reload");
        loadTrajectory("Sweeper");
        loadTrajectory("TwoCycleLeft");
        loadTrajectory("TwoCycleRight");
        loadTrajectory("CircularRabbit");
        loadTrajectory("HokeyPokeyRabbit");
        loadTrajectory("PoopingSloth");
        loadTrajectory("Tortoise");

        // Shoot-only trajectories
        loadTrajectory("left_shoot_only");
        loadTrajectory("mid_shoot_only");
        loadTrajectory("right_shoot_only");

        // Left-side cycle trajectories
        loadTrajectory("left_first_mid");
        loadTrajectory("left_first_depot");
        loadTrajectory("left_second_mid");
        loadTrajectory("left_second_depot");
        loadTrajectory("left_close_outpost");

        // Right-side cycle trajectories
        loadTrajectory("right_first_mid");
        loadTrajectory("right_first_outpost");
        loadTrajectory("right_second_mid");
        loadTrajectory("right_second_outpost");
        loadTrajectory("right_close_outpost");

        // Middle-start cycle trajectories (mid + mid is not valid; middle only has depot/outpost/shoot)
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


    private Command buildCommand(String trajectoryName) {
        if (trajectoryName == null) return Commands.none();
        switch (trajectoryName){
            case "Reload":
                return reloadAuto();
            case "TwoCycleRight", "TwoCycleLeft":
                return twoCycleAuto(trajectoryName);
            default:
                return basicAuto(trajectoryName);
        }
    }

    private Command stopDrive() {
        return Commands.runOnce(() -> drivetrainBase.drive(new ChassisSpeeds(), false), drivetrainBase);
    }

    private Command basicAuto(String name) {
        Command trajectory = choreoFactory.trajectoryCmd(name);
        Command printName = new PrintCommand("Auto: " + name);

        if (intake == null) return Commands.sequence(printName, trajectory, stopDrive());

        return Commands.sequence(
            printName,
            Commands.parallel(
                new IntakeHome(intake),
                trajectory
            ),
            stopDrive()
        );
    }

    private Command reloadAuto() {
        String name = "Reload";
        Command intakeLeg1 = choreoFactory.trajectoryCmd(name, 0);
        Command shootLeg1 = choreoFactory.trajectoryCmd(name, 1);
        Command printName = new PrintCommand("Auto: " + name);



        if (intake == null || shooterHopper == null) return Commands.sequence(printName, intakeLeg1, stopDrive());

        return Commands.sequence(
            printName,
            Commands.parallel(
                new IntakeHome(intake),
                intakeLeg1
            ),
            stopDrive(),
            Commands.parallel(
                new TimedShootCommand(shooterHopper, true,8),
                new DynamicRumble(intake)
            ),
            Commands.parallel(
                new IntakeHome(intake),
                shootLeg1
            ),
            stopDrive(),
            Commands.parallel(
                new TimedShootCommand(shooterHopper, true,8),
                new DynamicRumble(intake)
            )

        );
    }

    private Command twoCycleAuto(String name) {
        Command printName = new PrintCommand("Auto: " + name);

        Command intakeLeg1 = choreoFactory.trajectoryCmd(name, 0);
        Command shootLeg1 = choreoFactory.trajectoryCmd(name, 1);
        Command intakeLeg2 = choreoFactory.trajectoryCmd(name, 2);
        Command shootLeg2 = choreoFactory.trajectoryCmd(name, 3);

        return Commands.sequence(
            printName,
            Commands.parallel(
                new IntakeHome(intake),
                Commands.race(
                    intakeLeg1,
                    new RunRoller(intake, true)
                )
            ),
            stopDrive(),
            shootLeg1,
            stopDrive(),
            Commands.parallel(
                new TimedShootCommand(shooterHopper, true, 8.0),
                new FaceTowardHub(drivetrainBase),
                new DynamicRumble(intake)
            ),
            new IntakeGoToPosition(intake, IntakePosition.HOME),
            Commands.race(
                intakeLeg2,
                new RunRoller(intake, true)
            ),
            stopDrive(),
            shootLeg2,
            Commands.parallel(
                new TimedShootCommand(shooterHopper, true, 8.0),
                new DynamicRumble(intake)
            )
        );
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
        String firstTrajectory = resolveFirstTrajectory(start, firstCycle);
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
            Commands.race(
                Commands.sequence(
                    new IntakeHome(intake),
                    new IntakeGoToPosition(intake, IntakePosition.BOTTOM),
                    new RunRoller(intake, true)
                ),
                Commands.sequence(
                    new WaitCommand(0.2),
                    choreoFactory.trajectoryCmd(firstTrajectory)
                )
            )
        );
        commands.add(stopDrive());
        commands.add(Commands.parallel(
            new TimedShootCommand(shooterHopper, true, 8.0),
            new FaceTowardHub(drivetrainBase),
            new DynamicRumble(intake)
        ));

        // Second cycle (if selected)
        if (secondTrajectory != null) {
            commands.add(Commands.race(
                Commands.sequence(
                    new IntakeGoToPosition(intake, IntakePosition.BOTTOM),
                    new RunRoller(intake, true)
                ),
                choreoFactory.trajectoryCmd(secondTrajectory)
            ));
            commands.add(stopDrive());
            commands.add(Commands.parallel(
                new TimedShootCommand(shooterHopper, true, 8.0),
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
                new ShootCommand(shooterHopper, true),
                new DynamicRumble(intake)
            )
        );
    }

    private String resolveFirstTrajectory(String start, String firstCycle) {
        if ("close_outpost".equals(firstCycle)) {

            return start + "_close_outpost"; // left n right
        }
        return start + "_first_" + firstCycle;
    }


    private String resolveEffectiveSide(String start, String firstCycle) {
        if ("left".equals(start) || "right".equals(start)) return start;
        // Middle start: depot and mid end at left scoring position, outpost ends at right
        if ("outpost".equals(firstCycle)) return "right";
        return "left";
    }

    private String resolveSecondTrajectory(String effectiveSide, String secondCycle) {
        if ("none".equals(secondCycle)) return null;
        return effectiveSide + "_second_" + secondCycle;
    }

    private String resolveInitialPoseTrajectory(String start, String firstCycle) {
        if ("shoot_only".equals(firstCycle)) return start + "_shoot_only";
        if ("close_outpost".equals(firstCycle)) return start + "_close_outpost";
        return resolveFirstTrajectory(start, firstCycle);
    }


    private boolean isValidCycleCombo(String start, String firstCycle, String secondCycle) {

        if ("mid".equals(start) && "close_outpost".equals(firstCycle)) return false;

        // Middle can't do mid as first cycle
        if ("mid".equals(start) && "mid".equals(firstCycle)) return false;

        // Left can't do outpost
        if ("left".equals(start)) {
            if ("outpost".equals(firstCycle) || "outpost".equals(secondCycle)) return false;
        }

        // Right can't do depot
        if ("right".equals(start)) {
            if ("depot".equals(firstCycle) || "depot".equals(secondCycle)) return false;
        }

        // Middle: second cycle must match effective side
        if ("mid".equals(start) && !"none".equals(secondCycle) && !"shoot_only".equals(firstCycle)) {
            String effectiveSide = resolveEffectiveSide(start, firstCycle);
            if ("left".equals(effectiveSide) && "outpost".equals(secondCycle)) return false;
            if ("right".equals(effectiveSide) && "depot".equals(secondCycle)) return false;
        }

        return true;
    }
}
