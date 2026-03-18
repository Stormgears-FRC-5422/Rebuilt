package frc.robot.commands.diagnostic;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.IntakeHome;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterHopper;

public class SystemCheck extends SequentialCommandGroup {

    /** Runs all diagnostics indefinitely until cancelled. */
    public SystemCheck(ShooterHopper shooterHopper, Intake intake) {
        this(shooterHopper, intake, 0);
    }

    /** Runs all diagnostics deadlined by durationSeconds (0 = indefinite). */
    public SystemCheck(ShooterHopper shooterHopper, Intake intake, double durationSeconds) {
        // Home intake first
        if (intake != null) {
            addCommands(new IntakeHome(intake));
        }

        // All diagnostics run in parallel
        List<Command> diagCommands = new ArrayList<>();

        if (shooterHopper != null) {
            diagCommands.add(new ShooterDiagnostic(shooterHopper));
            diagCommands.add(
                Commands.sequence(
                    Commands.waitSeconds(1.0),
                    new HopperDiagnostic(shooterHopper)
                )
            );
        }

        if (intake != null) {
            diagCommands.add(new IntakePositionDiagnostic(intake));
            diagCommands.add(new IntakeRollerDiagnostic(intake));
        }

        if (!diagCommands.isEmpty()) {
            Command[] cmds = diagCommands.toArray(new Command[0]);
            if (durationSeconds > 0) {
                addCommands(new ParallelDeadlineGroup(Commands.waitSeconds(durationSeconds), cmds));
            } else {
                addCommands(new ParallelCommandGroup(cmds));
            }
        }
    }
}
