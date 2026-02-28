package frc.robot.commands.intake;

import frc.robot.Constants.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.utils.StormCommand;

public class DynamicRumble extends StormCommand {
    private final Intake intake;

    private double currentMinDeg;
    private int topTimes;

    public DynamicRumble(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        topTimes = 0;
        currentMinDeg = Shooter.minRumbleAngle;

        intake.setRumbleMinAngle(currentMinDeg);
        intake.requestRumble(); 
    }

    @Override
    public void execute() {
        if (intake.intakeIsAtMax()) {
            topTimes++; //good if increasing it every iteration is tooo much which i suspect it will be 
            currentMinDeg -= Shooter.dynamicRumbleIncrementDegrees;
            intake.setRumbleMinAngle(currentMinDeg);
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return !intake.hasBeenHomed || (intake.state != IntakeState.RUMBLE);
    }
}