package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.utils.StormCommand;

public class SimpleRumble extends StormCommand {
    private boolean skip;
    private final Intake intake;

    private double minAngle;
    private double maxAngle;
    private boolean goingUp;
    private edu.wpi.first.wpilibj.Timer timer = new Timer();

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
        minAngle = frc.robot.Constants.Intake.minRumbleAngleDeg;
        maxAngle = frc.robot.Constants.Intake.maxRumbleAngleDeg;
        goingUp = true;

        intake.setMotionProfile(Intake.MotionProfile.AUTO);
        intake.setTargetPosition(maxAngle);
        intake.setState(IntakeState.PID_MOTION);
    }

    @Override
    public void execute() {
        super.execute();
        if (skip) return;
        if (intake.isAtTarget() || timer.hasElapsed(0.4)) {
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
