package frc.robot.commands.intake;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakePosition;
import frc.robot.subsystems.Intake.IntakeState;
import frc.utils.StormCommand;

public class IntakeGoToPosition extends StormCommand {
    private boolean skip;
    private final Intake intake;
    private final double degrees;

    public IntakeGoToPosition(Intake intake, IntakePosition position) {
        this(intake, position.getValue());
    }

    public IntakeGoToPosition(Intake intake, double degrees) {
        this.intake = intake;
        this.degrees = degrees;
        skip = safeAddRequirements(intake);
    }

    @Override
    public void initialize() {
        super.initialize();
        if (skip) return;
        intake.setTargetPosition(degrees);
        intake.setState(IntakeState.PID_MOTION);
    }

    @Override
    public void execute() {
        super.execute();
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
