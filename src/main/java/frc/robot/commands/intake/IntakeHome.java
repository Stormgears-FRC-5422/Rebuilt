package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.utils.StormCommand;

public class IntakeHome extends StormCommand {
    private final boolean skip;   // permanent: true if requirements couldn't be added
    private boolean alreadyHomed; // per-run: reset each initialize()
    private final Intake intake;

    public IntakeHome(Intake intake) {
        this.intake = intake;
        skip = safeAddRequirements(intake);
    }

    @Override
    public void initialize() {
        super.initialize();
        alreadyHomed = false;
        if (skip) return;

        alreadyHomed = !frc.robot.Constants.Intake.allowRehome && intake.hasBeenHomed;
        if (!alreadyHomed) {
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
        return skip || alreadyHomed || intake.isAtHome();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (!skip && !alreadyHomed) {
            if (!interrupted) {
                intake.declareHome();
                intake.holdCurrentPosition(0.0);
            } else {
                intake.setState(IntakeState.UNKNOWN);
            }
        }
    }
}
