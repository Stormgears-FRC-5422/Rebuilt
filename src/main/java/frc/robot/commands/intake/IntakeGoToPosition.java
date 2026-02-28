package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeGoToPosition extends Command {
    private final Intake intake;
    private final double degrees;

    public IntakeGoToPosition(Intake intake, double degrees) {
        this.intake = intake;
        this.degrees = degrees;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.requestPidMotion(degrees);
    }

    @Override
    public boolean isFinished() {
        return intake.atTargetDegrees(degrees);
    }

    @Override
    public void end(boolean interrupted) {
    }
}