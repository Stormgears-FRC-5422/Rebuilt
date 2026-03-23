package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ShooterHopper;
import frc.robot.subsystems.ShooterHopper.ShooterState;
import frc.utils.StormCommand;

/**
 * Shoots until the hopper is empty (detected via flywheel velocity dip).
 * For auto use only — teleop uses ShootCommand (which never self-terminates).
 */
public class AutoShootCommand extends StormCommand {

    private final ShooterHopper shooter;
    private final Timer timer = new Timer();
    private final boolean toHub;

    public AutoShootCommand(ShooterHopper shooter, boolean toHub) {
        this.shooter = shooter;
        this.toHub   = toHub;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        shooter.setToHub(toHub);
        shooter.setState(ShooterState.SPEEDING_UP); // also resets hopper detection
    }

    @Override
    public void execute() {
        if (shooter.isReadyToFeed()) {
            shooter.setState(ShooterState.SHOOTING);
        }
    }

    @Override
    public boolean isFinished() {
        return shooter.isHopperEmpty();
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        shooter.setState(ShooterState.IDLE);
    }
}
