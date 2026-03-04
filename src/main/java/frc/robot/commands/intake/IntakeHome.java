package frc.robot.commands.intake;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.utils.StormCommand;

public class IntakeHome extends StormCommand {
    // Set to true to allow re-homing even if already homed
    private static final boolean ALLOW_REHOME = false;

    private boolean skip;
    private final Intake intake;

    public IntakeHome(Intake intake) {
        this.intake = intake;
        skip = safeAddRequirements(intake);
    }

    @Override
    public void initialize() {
        super.initialize();
        if (skip) return;

        skip = skip || (!ALLOW_REHOME && intake.hasBeenHomed);
        if (!skip) {
            intake.setState(IntakeState.HOMING);
        } else {
            console("skipping intake home - already homed");
        }
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return skip || intake.isAtHome();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (!skip) {
            if (!interrupted) {
                intake.declareHome();
                intake.holdCurrentPosition();
            } else {
                intake.setState(IntakeState.UNKNOWN);
            }
        }
    }
}
