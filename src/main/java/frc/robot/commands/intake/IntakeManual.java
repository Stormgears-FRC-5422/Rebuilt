package frc.robot.commands.intake;

import frc.robot.Constants;
import frc.robot.joysticks.RebuiltJoystick;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.utils.StormCommand;

public class IntakeManual extends StormCommand {
    private final boolean skip;
    private final Intake intake;
    private final RebuiltJoystick joystick;

    enum Direction {HOLD, UP, DOWN}

    private Direction previousDirection = Direction.HOLD;

    public IntakeManual(Intake intake, RebuiltJoystick joystick) {
        this.intake = intake;
        this.joystick = joystick;
        skip = safeAddRequirements(intake);
    }

    @Override
    public void initialize() {
        super.initialize();
        previousDirection = Direction.HOLD;
        if (skip) return;
        intake.setMotionProfile(Intake.MotionProfile.MANUAL);
        if (intake.hasBeenHomed) {
            intake.setTargetPosition(intake.getCurrentPosition());
        }
        intake.setState(IntakeState.HOLD);
    }


    @Override
    public void execute() {
        super.execute();
        if (skip || !intake.hasBeenHomed) return;

        Direction direction = joystick.intakeUp() ? Direction.UP
            : joystick.intakeDown() ? Direction.DOWN
            : Direction.HOLD;

        if (direction != previousDirection) {
            if (direction == Direction.UP) {
                intake.setTargetPosition(Constants.Intake.topIntakeDegrees);
                intake.setState(IntakeState.PID_MOTION);
            } else if (direction == Direction.DOWN) {
                intake.setTargetPosition(Constants.Intake.intakingAngle);
                intake.setState(IntakeState.PID_MOTION);
            } else {
                intake.setTargetPosition(intake.getCurrentPosition());
                intake.setState(IntakeState.HOLD);
            }
            previousDirection = direction;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (!skip) {
            intake.setMotionProfile(Intake.MotionProfile.AUTO);
            if (intake.hasBeenHomed) {
                intake.holdCurrentPosition();
            }
        }
        super.end(interrupted);
    }
}
