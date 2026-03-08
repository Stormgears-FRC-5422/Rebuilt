package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
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
    private int maxCycles;
    private edu.wpi.first.wpilibj.Timer timer = new Timer();

    public DynamicRumble(Intake intake) {
        this.intake = intake;
        skip = safeAddRequirements(intake);
    }

    @Override
    public void initialize() {
        super.initialize();
        timer.reset();
        timer.start();

        if (skip) return;
        skip = !intake.hasBeenHomed;
        if (skip) return;

        minAngle = frc.robot.Constants.Intake.minRumbleAngleDeg;
        maxAngle = frc.robot.Constants.Intake.maxRumbleAngleDeg;
        goingUp = true;
        counter = 0;

        double testMinAngle = minAngle;
        double safeFloor = frc.robot.Constants.Intake.topIntakeLimit;
        double normalStep = frc.robot.Constants.Intake.dynamicRumbleIncrementDegrees;

        maxCycles = 0;
        while (true) {
            double step = (maxCycles < 3) ? 25.0 : normalStep;
            if (testMinAngle - step <= safeFloor) {
                break;
            }
            testMinAngle -= step;
            maxCycles++;
        }

        intake.setMotionProfile(Intake.MotionProfile.AUTO);
        intake.setTargetPosition(maxAngle);
        intake.setState(IntakeState.PID_MOTION);
    }

    @Override
    public void execute() {
        super.execute();

        if (skip) return;

        if (intake.isAtTarget() || timer.hasElapsed(0.75)) {
            if (goingUp && counter >= maxCycles) {
                return;
            }

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
            timer.restart();
        }
    }

    @Override
    public boolean isFinished() {
        return skip
            || !intake.hasBeenHomed
            || intake.state != IntakeState.PID_MOTION
            || (goingUp && intake.isAtTarget() && counter >= maxCycles);
    }

    @Override
    public void end(boolean interrupted) {
        if (!skip) {
            intake.setRollerEnabled(false);
            intake.holdCurrentPosition();
        }
        super.end(interrupted);
    }
}
