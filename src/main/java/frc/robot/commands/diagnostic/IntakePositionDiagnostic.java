package frc.robot.commands.diagnostic;

import frc.robot.Options.IntakeOptions;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Intake.MotionProfile;
import frc.utils.StormCommand;

public class IntakePositionDiagnostic extends StormCommand {
    private final Intake intake;
    private final boolean skip;

    public IntakePositionDiagnostic(Intake intake) {
        this.intake = intake;
        skip = safeAddRequirements(intake);
    }

    @Override
    public void initialize() {
        super.initialize();
        if (skip || !intake.hasBeenHomed) {
            if (!skip) console("Skipping — intake not homed");
            return;
        }
        double position = IntakeOptions.create().diagnosticPosition.get();
        intake.setMotionProfile(MotionProfile.MANUAL);
        intake.setTargetPosition(position);
        intake.setState(IntakeState.PID_MOTION);
        console("Intake position diagnostic — PID_MOTION (MANUAL) to " + position + "°");
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return skip || !intake.hasBeenHomed;
    }

    @Override
    public void end(boolean interrupted) {
        if (!skip && intake.hasBeenHomed) {
            intake.holdCurrentPosition();
        }
        super.end(interrupted);
    }
}
