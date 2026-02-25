package frc.robot.commands;

import frc.robot.Constants.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.utils.StormCommand;

public class DynamicRumble extends StormCommand {
    private final Intake intake;

    private double currentMaxDeg;
    private int topTimes;

    public DynamicRumble(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        topTimes = 0;
        currentMaxDeg = Shooter.maxRumbleAngle;

        intake.setRumbleMaxAngle(currentMaxDeg);
        intake.requestRumble(); 
    }

    @Override
    public void execute() {
        if (intake.intakeIsAtMax()) {
            topTimes++; //good if increasing it every iteration is tooo much which i suspect it will be 
            currentMaxDeg += Shooter.dynamicRumbleIncrementDegrees;
            intake.setRumbleMaxAngle(currentMaxDeg);
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