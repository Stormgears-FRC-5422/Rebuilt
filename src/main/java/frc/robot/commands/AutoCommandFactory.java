package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterHopper;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.Constants.Auto;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.Objects;
import java.util.Optional;

public class AutoCommandFactory {
    private final DrivetrainBase drivetrainBase;
    private final ShooterHopper shooterHopper;
    private final Intake intake;
    private final RobotState robotState;

    private AutoFactory choreoFactory;
    private final LoggedDashboardChooser<String> autoChooser = new LoggedDashboardChooser<>("Auto Routine");

    @AutoLogOutput
    private boolean isDirty;
    private boolean chooserInitialized = false;
    private Command chosenCommand;
    @SuppressWarnings("OptionalUsedAsFieldOrParameterType")
    private Optional<? extends Trajectory<?>> chosenTrajectory = Optional.empty();
    private String chooserTrajectoryName = Auto.path;

    public AutoCommandFactory(DrivetrainBase drivetrainBase, ShooterHopper shooterHopper, Intake intake) {
        this.robotState = RobotState.getInstance();
        this.drivetrainBase = drivetrainBase;
        this.shooterHopper = shooterHopper;
        this.intake = intake;
    }

    /**
     * Must be called after construction and after alliance color is known.
     * Creates the Choreo factory, caches trajectories, and sets up the chooser.
     */
    public void initialize() {
        choreoFactory = new AutoFactory(
            drivetrainBase::getPose,
            drivetrainBase::setPose,
            drivetrainBase::followTrajectory,
            robotState.isAllianceRed(),
            drivetrainBase
        );

        loadTrajectories();

        // Build chooser options only once — addOption accumulates duplicates on repeated calls
        if (!chooserInitialized) {
            autoChooser.addDefaultOption(Auto.path, Auto.path);
            autoChooser.addOption("Nothing", null);
            autoChooser.addOption("Straight", "Straight");
            autoChooser.addOption("Reload", "Reload");
            autoChooser.addOption("Sweeper", "Sweeper");
            autoChooser.addOption("TwoCycleLeft", "TwoCycleLeft");
            autoChooser.addOption("TwoCycleRight", "TwoCycleRight");
            chooserInitialized = true;
        }

        setDirty();
    }

    /* **********
     * Chooser access
     * **********/

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
        // isDirty is set by initialize() on every alliance change (even if same selection).
        // The Objects.equals check additionally catches selection changes between alliance updates.
        if (!isDirty && Objects.equals(currentSelection, chooserTrajectoryName)) return;

        chooserTrajectoryName = currentSelection;
        chosenCommand = buildCommand(chooserTrajectoryName);
        chosenTrajectory = (chooserTrajectoryName != null)
            ? loadTrajectory(chooserTrajectoryName)
            : Optional.empty();

        isDirty = false;
    }

    /* **********
     * Trajectory loading
     * **********/
    private void loadTrajectories() {
        loadTrajectory("Straight");
        loadTrajectory("Reload");
        loadTrajectory("Sweeper");
        loadTrajectory("TwoCycleLeft");
        loadTrajectory("TwoCycleRight");
    }

    private Optional<? extends Trajectory<?>> loadTrajectory(String trajectoryName) {
        return choreoFactory.cache().loadTrajectory(trajectoryName);
    }

    /* **********
     * Command building
     * **********/
    private Command buildCommand(String trajectoryName) {
        if (trajectoryName == null) return Commands.none();
        return basicAuto(trajectoryName);
    }

    /**
     * Runs a named trajectory while homing the intake in parallel.
     * Additional per-path complexity (e.g. shooting, scoring) can be layered on top of this.
     */
    private Command basicAuto(String name) {
        Command trajectory = choreoFactory.trajectoryCmd(name);
        Command printName = new PrintCommand("Auto: " + name);

        if (intake == null) return Commands.sequence(printName, trajectory);

        return Commands.sequence(
            printName,
            Commands.parallel(
                Commands.runOnce(intake::requestHome, intake),
                trajectory
            )
        );
    }
}
