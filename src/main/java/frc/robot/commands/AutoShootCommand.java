package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.RobotState.HopperState;
import frc.robot.subsystems.ShooterHopper;
import frc.robot.subsystems.ShooterHopper.ShooterState;
import frc.utils.StormCommand;

/**
 * Shoots until the hopper is empty, determined by fusing two independent signals:
 *   1. Flywheel disturbance detector — flywheel spinning clean with no velocity dips
 *   2. Physical sensors (beam break + lidar) via RobotState.getHopperState()
 * Either signal alone is sufficient to stop. For auto use only — teleop uses
 * ShootCommand (which never self-terminates).
 */
public class AutoShootCommand extends StormCommand {

    private final ShooterHopper shooter;
    private final RobotState robotState;
    private final Timer timer = new Timer();
    private final boolean toHub;

    public AutoShootCommand(ShooterHopper shooter, boolean toHub) {
        this.shooter    = shooter;
        this.robotState = RobotState.getInstance();
        this.toHub      = toHub;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        shooter.setToHub(toHub);
        shooter.setState(ShooterState.SPEEDING_UP); // also resets flywheel detection
    }

    @Override
    public void execute() {
        if (shooter.isReadyToFeed()) {
            shooter.setState(ShooterState.SHOOTING);
        }
    }

    @Override
    public boolean isFinished() {
        return !shooter.isFlywheelBusy()
            || robotState.getHopperState() == HopperState.HOPPER_EMPTY;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        shooter.setState(ShooterState.IDLE);
    }
}
