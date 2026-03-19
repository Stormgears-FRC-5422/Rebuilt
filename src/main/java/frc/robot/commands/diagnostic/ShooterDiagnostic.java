package frc.robot.commands.diagnostic;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ShooterHopper;
import frc.robot.subsystems.ShooterHopper.ShooterState;
import frc.utils.StormCommand;

public class ShooterDiagnostic extends StormCommand {
    private final ShooterHopper shooter;
    private final BooleanSupplier runShooterSupplier;
    private final BooleanSupplier runHopperSupplier;
    private final boolean skip;

    private boolean runShooter;
    private boolean runHopper;
    private final Timer hopperDelayTimer = new Timer();
    private boolean hopperStarted;

    public ShooterDiagnostic(ShooterHopper shooter, BooleanSupplier runShooter, BooleanSupplier runHopper) {
        this.shooter = shooter;
        this.runShooterSupplier = runShooter;
        this.runHopperSupplier = runHopper;
        skip = safeAddRequirements(shooter);
    }

    @Override
    public void initialize() {
        super.initialize();
        if (skip) return;

        runShooter = runShooterSupplier.getAsBoolean();
        runHopper = runHopperSupplier.getAsBoolean();
        hopperStarted = false;

        if (runShooter) {
            shooter.setState(ShooterState.SPEEDING_UP);
            console("Shooter diagnostic — wheels spinning up");
            if (runHopper) {
                hopperDelayTimer.restart();
            }
        } else if (runHopper) {
            shooter.setState(ShooterState.DIAGNOSTIC);
            hopperStarted = true;
            console("Shooter diagnostic — fuel injector + shooter wheels");
        }
    }

    @Override
    public void execute() {
        super.execute();
        if (skip) return;

        if (runShooter) {
            console("Shooter velocity: " + shooter.getCurrentVelocityRPS() + " RPS", 50);
        }

        if (runShooter && runHopper && !hopperStarted && hopperDelayTimer.hasElapsed(1.0)) {
            shooter.setState(ShooterState.DIAGNOSTIC);
            hopperStarted = true;
            console("Shooter diagnostic — engaging fuel injector");
        }
    }

    @Override
    public boolean isFinished() {
        return skip || (!runShooter && !runHopper);
    }

    @Override
    public void end(boolean interrupted) {
        if (!skip) {
            shooter.setState(ShooterState.IDLE);
            hopperDelayTimer.stop();
        }
        super.end(interrupted);
    }
}
