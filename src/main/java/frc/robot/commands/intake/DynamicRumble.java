package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.utils.StormCommand;

public class DynamicRumble extends StormCommand {
    private boolean skip;
    private boolean done;
    private final Intake intake;

    private final double minRumbleAngle = Constants.Intake.minRumbleAngleDeg;
    private final double maxAngle = Constants.Intake.maxRumbleAngleDeg;
    private final double safeFloor = Constants.Intake.topIntakeLimit;
    private final double normalStep = Constants.Intake.dynamicRumbleIncrementDegrees;
    private final double bigStep = Constants.Intake.dynamicRumbleBigStep;
    private final int bigStepCycles = Constants.Intake.dynamicRumbleBigStepCycles;
    private final double timeout = Constants.Intake.dynamicRumbleTimeout;

    private double minAngle;
    private boolean goingUp;
    private int counter;
    private int maxCycles;
    private final Timer timer = new Timer();

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

        minAngle = minRumbleAngle;
        goingUp = true;
        counter = 0;
        done = false;

        double testMinAngle = minAngle;
        maxCycles = 0;
        while (true) {
            double step = (maxCycles < bigStepCycles) ? bigStep : normalStep;
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

        if (intake.isAtTarget() || timer.hasElapsed(timeout)) {
            if (goingUp && counter >= maxCycles) {
                done = true;
                return;
            }

            if (goingUp) {
                if (counter < bigStepCycles) {
                    minAngle -= bigStep;
                } else {
                    minAngle -= normalStep;
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
            || done
            || !intake.hasBeenHomed
            || intake.state != IntakeState.PID_MOTION;
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
