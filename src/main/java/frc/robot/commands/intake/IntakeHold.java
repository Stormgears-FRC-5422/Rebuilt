package frc.robot.commands.intake;

import frc.robot.subsystems.Intake;
import frc.utils.StormCommand;

public class IntakeHold extends StormCommand {
    private final Intake intake;
    private final boolean skip;

    private final boolean holdAtCurrentPosition;
    private final double holdPosition;

    public IntakeHold(Intake intake) {
        this.intake = intake;
        this.skip = safeAddRequirements(intake);
        this.holdAtCurrentPosition = true;
        this.holdPosition = 0.0;
    }

    public IntakeHold(Intake intake, Intake.IntakePosition targetPosition) {
        this(intake, targetPosition.getValue());
    }

    public IntakeHold(Intake intake, double position) {
        this.intake = intake;
        this.skip = safeAddRequirements(intake);
        this.holdAtCurrentPosition = false;
        this.holdPosition = position;
    }

    @Override
    public void initialize() {
        super.initialize();
        if (skip) return;

        intake.setTargetPosition(holdAtCurrentPosition ? intake.getCurrentPosition() : holdPosition);
        intake.setState(Intake.IntakeState.HOLD);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}