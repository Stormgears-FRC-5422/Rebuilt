package frc.robot.commands;

import frc.robot.subsystems.Intake;
import frc.utils.StormCommand;

public class IntakeUp extends StormCommand {
    private final Intake intake;

    public IntakeUp(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.requestIdle();
        intake.setRollerEnabled(false);
    }

    @Override
    public boolean isFinished() {
        return intake.atTargetDegrees(1.0); 
    }

    @Override
    public void end(boolean interrupted) {}
}