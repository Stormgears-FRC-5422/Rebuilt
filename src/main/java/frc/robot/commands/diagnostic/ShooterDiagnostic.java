package frc.robot.commands.diagnostic;

import frc.robot.subsystems.ShooterHopper;
import frc.robot.subsystems.ShooterHopper.ShooterState;
import frc.utils.StormCommand;

public class ShooterDiagnostic extends StormCommand {
    private final ShooterHopper shooter;
    private final boolean skip;

    public ShooterDiagnostic(ShooterHopper shooter) {
        this.shooter = shooter;
        skip = safeAddRequirements(shooter);
    }

    @Override
    public void initialize() {
        super.initialize();
        if (skip) return;
        shooter.setState(ShooterState.SPEEDING_UP);
        console("Shooter diagnostic — wheels only, no fuel injector");
    }

    @Override
    public void execute() {
        super.execute();
        if (!skip) {
            console("Shooter velocity: " + shooter.getCurrentVelocityRPS() + " RPS", 50);
        }
    }

    @Override
    public boolean isFinished() {
        return skip;
    }

    @Override
    public void end(boolean interrupted) {
        if (!skip) {
            shooter.setState(ShooterState.IDLE);
        }
        super.end(interrupted);
    }
}
