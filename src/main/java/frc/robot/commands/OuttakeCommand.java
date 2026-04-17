package frc.robot.commands;

import frc.robot.subsystems.ShooterHopper;
import frc.robot.subsystems.ShooterHopper.ShooterState;
import frc.utils.StormCommand;

public class OuttakeCommand extends StormCommand {

    private final ShooterHopper shooter;

    public OuttakeCommand(ShooterHopper shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        super.initialize();
        shooter.setState(ShooterState.REVERSE);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setState(ShooterState.IDLE);
        super.end(interrupted);
    }
}
