package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeDown extends Command {
    private final Intake intake;

    public IntakeDown(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.requestIntakeDown();
        intake.setRollerEnabled(true);
    }

    @Override
    public boolean isFinished() {
        return intake.atTargetDegrees(intake.elasticMotorController.intakeAngle.get().doubleValue());
    }

    @Override
    public void end(boolean interrupted) {}
}