package frc.robot.commands.intake;

import frc.robot.Constants.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.utils.StormCommand;

public class RunIntake extends StormCommand {
    private final Intake intake;

    public RunIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        super.initialize();
        intake.setRollerEnabled(true);
    }
    @Override
    public void execute() {
        super.execute();
        System.out.println("trying to intake");
    }

    @Override
    public void end(boolean interrupted) {
        intake.setRollerEnabled(false);
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

