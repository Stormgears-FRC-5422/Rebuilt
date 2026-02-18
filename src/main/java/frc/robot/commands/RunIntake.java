package frc.robot.commands;

import frc.robot.subsystems.Intake;
import frc.utils.StormCommand;

public class RunIntake extends StormCommand{
    private final Intake intake;
    private final boolean requestIntakingState;

    public RunIntake(Intake intake) {
        this(intake, false);
    }

    public RunIntake(Intake intake, boolean requestIntakingState) {
        this.intake = intake;
        this.requestIntakingState = requestIntakingState;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (requestIntakingState) {
            intake.requestIntakeDown();
        }
        intake.setRollerEnabled(true);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        intake.setRollerEnabled(false);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
