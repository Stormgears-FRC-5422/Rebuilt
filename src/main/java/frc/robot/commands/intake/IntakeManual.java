package frc.robot.commands.intake;

import frc.robot.joysticks.RebuiltJoystick;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.utils.StormCommand;

public class IntakeManual extends StormCommand {
    private final boolean skip;
    private final Intake intake;
    private final RebuiltJoystick joystick;

    enum Direction { HOLD, UP, DOWN }

    public IntakeManual(Intake intake, RebuiltJoystick joystick) {
        this.intake = intake;
        this.joystick = joystick;
        skip = safeAddRequirements(intake);
    }

    @Override
    public void initialize() {
        super.initialize();
        if (skip) return;
        intake.setTargetPosition(intake.getCurrentPosition());
        intake.setState(IntakeState.HOLD);
    }

    @Override
    public void execute() {
        super.execute();
        if (skip || !intake.hasBeenHomed) return;

        double currentPosition = intake.getCurrentPosition();

        Direction direction = joystick.intakeUp() ? Direction.UP
            : joystick.intakeDown() ? Direction.DOWN
            : Direction.HOLD;

        if (direction == Direction.UP) {
            intake.setTargetPosition(frc.robot.Constants.Intake.topIntakeDegrees);
            intake.setState(IntakeState.PID_MOTION);
        } else if (direction == Direction.DOWN) {
            intake.setTargetPosition(frc.robot.Constants.Intake.intakingAngle);
            intake.setState(IntakeState.PID_MOTION);
        } else {
            intake.setTargetPosition(currentPosition);
            intake.setState(IntakeState.HOLD);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (!skip) {
            intake.holdCurrentPosition();
        }
        super.end(interrupted);
    }
}
