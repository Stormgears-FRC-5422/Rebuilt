package frc.robot.commands.climber;

import frc.robot.subsystems.Climber;
import frc.utils.StormCommand;

public class ClimberHome extends StormCommand {
    private final Climber climber;

    public ClimberHome(Climber climber) {
        this.climber = climber;
        safeAddRequirements(climber);
    }

    @Override
    public void initialize() {
        super.initialize();
        climber.zeroHere();
        climber.setState(Climber.ClimberState.IDLE);
        climber.hasBeenHomed = true;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
