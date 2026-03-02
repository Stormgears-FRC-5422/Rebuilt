package frc.robot.commands.intake;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakePosition;
import frc.utils.StormCommand;

public class IntakeDiagnostic extends StormCommand {
    private boolean skip;

    private Intake intake;
    private boolean down;

    public IntakeDiagnostic(Intake intake, boolean down) {
        this.intake = intake;
        this.down = down;

        skip = safeAddRequirements(intake);
    }

    @Override
    public void initialize() {
        super.initialize();
        if (skip){
            return;
        }
        intake.setTargetPosition(down ? IntakePosition.BOTTOM : IntakePosition.TOP);
        intake.setState(Intake.IntakeState.SIMPLE_MOTION);
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return skip;
    }

    @Override
    public void end(boolean interrupted) {
        if (!skip){
            intake.setRollerEnabled(false);
            intake.setState(Intake.IntakeState.IDLE);
        } else if (intake != null){
            intake.setState(Intake.IntakeState.UNKNOWN);
        }
        super.end(interrupted);
    }
}

