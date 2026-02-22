package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ShooterHopper;
import frc.robot.subsystems.ShooterHopper.ShooterState;
import frc.utils.StormCommand;

public class TimedShootCommand extends StormCommand {

    private final ShooterHopper shooter;
    private final boolean useRealDist;
    private final double duration;
    private final Timer timer = new Timer();

    public TimedShootCommand(ShooterHopper shooter, boolean useRealDist, double durationSeconds) {
        this.shooter = shooter;
        this.useRealDist = useRealDist;
        this.duration = durationSeconds;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setUseRealDist(useRealDist);
        timer.reset();
        timer.start();
        shooter.setState(ShooterState.SPEEDING_UP);
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(1.0)) {
            shooter.setState(ShooterState.SHOOTING);
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        shooter.setState(ShooterState.IDLE);
    }
}
