package frc.robot.commands.diagnostic;

import frc.robot.subsystems.Intake;
import frc.utils.StormCommand;

// Does not require Intake subsystem so it can run in parallel with position commands
public class IntakeRollerDiagnostic extends StormCommand {
    private final Intake intake;
    private final boolean skip;

    public IntakeRollerDiagnostic(Intake intake) {
        this.intake = intake;
        skip = (intake == null);
    }

    @Override
    public void initialize() {
        super.initialize();
        if (skip) return;
        intake.setRollerDiagnostic(true);
        intake.setRollerEnabled(true);
        console("Intake roller diagnostic — roller bypass enabled");
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
            intake.setRollerDiagnostic(false);
        }
        super.end(interrupted);
    }
}
