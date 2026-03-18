package frc.robot.commands.diagnostic;

import frc.robot.subsystems.ShooterHopper;
import frc.robot.subsystems.ShooterHopper.ShooterState;
import frc.utils.StormCommand;

// Does not require ShooterHopper subsystem so it can run in parallel with ShooterDiagnostic
public class HopperDiagnostic extends StormCommand {
    private final ShooterHopper shooter;
    private final boolean skip;

    public HopperDiagnostic(ShooterHopper shooter) {
        this.shooter = shooter;
        skip = (shooter == null);
    }

    @Override
    public void initialize() {
        super.initialize();
        if (skip) return;
        shooter.setState(ShooterState.DIAGNOSTIC);
        console("Hopper diagnostic — fuel injector + shooter wheels");
    }

    @Override
    public void execute() {
        super.execute();
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
