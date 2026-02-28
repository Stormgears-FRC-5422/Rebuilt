package frc.robot.commands.intake;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakePositions;
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
        if (skip){
            return;
        }
        intake.setTargetPosition(down ? IntakePositions.BOTTOM : IntakePositions.TOP);
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
            intake.setState(Intake.IntakeState.IDLE);
        } else if (intake != null){
            intake.setState(Intake.IntakeState.UNKNOWN);
        }
        super.end(interrupted);
    }
}

