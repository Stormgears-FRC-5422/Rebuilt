package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.utils.StormCommand;

public class SimpleRumble extends StormCommand {
    private boolean skip;
    private final Intake intake;

    private final double minAngle = Constants.Intake.minRumbleAngleDeg;
    private final double maxAngle = Constants.Intake.maxRumbleAngleDeg;
    private final double timeout = Constants.Intake.simpleRumbleTimeout;

    private boolean goingUp;
    private final Timer timer = new Timer();

    public SimpleRumble(Intake intake) {
        this.intake = intake;
        skip = safeAddRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        super.initialize();
        if (skip) return;
        skip = !intake.hasBeenHomed;
        if (skip) return;
        goingUp = true;

        intake.setMotionProfile(Intake.MotionProfile.AUTO);
        intake.setTargetPosition(maxAngle);
        intake.setState(IntakeState.PID_MOTION);
    }

    @Override
    public void execute() {
        super.execute();
        if (skip) return;
        if (intake.isAtTarget() || timer.hasElapsed(timeout)) {
            goingUp = !goingUp;
            intake.setTargetPosition(goingUp ? maxAngle : minAngle);
            timer.restart();
        }
    }

    @Override
    public boolean isFinished() {
        return skip;
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
