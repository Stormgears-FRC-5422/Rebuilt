package frc.robot.commands.intake;

import frc.robot.subsystems.Intake;
import frc.utils.StormCommand;

// Does not require intake subsystem so it can run in parallel with motion commands
public class RunRoller extends StormCommand {
    private final Intake intake;
    private final boolean forward;
    private final boolean skip;

    public RunRoller(Intake intake, boolean forward) {
        this.intake = intake;
        this.forward = forward;
        skip = (intake == null);
    }

    @Override
    public void initialize() {
        super.initialize();
        if (skip) return;
        if (forward) {
            intake.setRollerEnabled(true);
        } else {
            intake.setOuttakeRollers(true);
        }
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return skip;
    }

    @Override
    public void end(boolean interrupted) {
        if (!skip) {
            intake.setRollerEnabled(false);
            intake.setOuttakeRollers(false);
        }
        super.end(interrupted);
    }
}
