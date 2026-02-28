package frc.robot.commands.intake;

import frc.robot.RobotState;
import frc.robot.subsystems.Intake;
import frc.utils.StormCommand;

public class IntakeHome extends StormCommand {
    private boolean skip;
    private int num;
    private final Intake intake;
    private RobotState robotState;
    // Don't allow homing if already homed. At least for now.
    // At this point the home will happen automatically at the beginning of auto or teleop
    // whichever is run first.

    public IntakeHome(Intake intake) {
        this.intake = intake;
        robotState = RobotState.getInstance();
        skip = safeAddRequirements(intake);
    }

    @Override
    public void initialize() {
        super.initialize();
        if (skip){
            return;
        }
        // skip = skip || intake.hasBeenHomed;
        if (!skip) {
            intake.setState(Intake.IntakeState.HOMING);
        } else {
            console("skipping intake home - already homed");
        }
    }

    @Override
    public void execute() {
        super.execute();

        if (intake.isAtHome()) {
            num++;
        } else {
            num = 0;
        }
    }

    @Override
    public boolean isFinished() {
        return skip || num > 15;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        intake.updownIntakeMotor.set(0);
        if (!skip) {
            if (!interrupted) {
                intake.setState(Intake.IntakeState.IDLE);
                intake.hasBeenHomed = true;
                intake.zeroHere();
                intake.enableSoftLimit(true);
           } else {
                intake.setState(Intake.IntakeState.UNKNOWN);
            }
        }

    }
}
