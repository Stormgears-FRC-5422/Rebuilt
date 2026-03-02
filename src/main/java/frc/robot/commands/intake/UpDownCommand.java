package frc.robot.commands.intake;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.utils.StormCommand;

public class UpDownCommand extends StormCommand {
    private boolean skip;
    private final Intake intake;

    private static boolean goToIntakingAngle = true;

    public UpDownCommand(Intake intake) {
        this.intake = intake;
        skip = safeAddRequirements(intake);
    }

    @Override
    public void initialize() {
        super.initialize();
        if (skip) return;

        double target = goToIntakingAngle
            ? frc.robot.Constants.Intake.intakingAngle
            : frc.robot.Constants.Intake.topIntakeDegrees;

        intake.setTargetPosition(target);
        intake.setState(IntakeState.PID_MOTION);

        goToIntakingAngle = !goToIntakingAngle;
    }

    @Override
    public boolean isFinished() {
        return skip || intake.isAtTarget();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}