package frc.robot.commands.intake;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.utils.StormCommand;

public class DynamicRumble extends StormCommand {
    private boolean skip;
    private final Intake intake;

    private double minAngle;
    private double maxAngle;
    private boolean goingUp;
    private int counter;

    public DynamicRumble(Intake intake) {
        this.intake = intake;
        skip = safeAddRequirements(intake);
    }

    @Override
    public void initialize() {
        super.initialize();
        if (skip) return;

        minAngle = frc.robot.Constants.Intake.minRumbleAngleDeg;
        maxAngle = frc.robot.Constants.Intake.maxRumbleAngleDeg;
        goingUp = true;
        counter = 0;

        intake.setTargetPosition(maxAngle);
        intake.setState(IntakeState.PID_MOTION);
    }

    @Override
    public void execute() {
        super.execute();
        if (skip) return;

        if (intake.isAtTarget()) {
            if (goingUp) {
                if (counter < 3) {
                    minAngle -= 25.0;
                } else {
                    minAngle -= frc.robot.Constants.Intake.dynamicRumbleIncrementDegrees;
                }
                counter++;
            }

            goingUp = !goingUp;
            intake.setTargetPosition(goingUp ? maxAngle : minAngle);
        }
    }

    @Override
    public boolean isFinished() {
        return skip
            || !intake.hasBeenHomed
            || (intake.state != IntakeState.PID_MOTION
            || (intake.updownIntakeMotor.getPosition().getValueAsDouble() < 20.0));
    }

    @Override
    public void end(boolean interrupted) {
        if (!skip) {
            intake.setRollerEnabled(false);
            intake.setState(IntakeState.IDLE);
        }
        super.end(interrupted);
    }
}