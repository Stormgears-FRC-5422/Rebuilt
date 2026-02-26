package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ShooterHopper;
import frc.robot.subsystems.ShooterHopper.ShooterState;
import frc.utils.StormCommand;

public class ShootCommand extends StormCommand {

    private final ShooterHopper shooter;
    private final Timer timer = new Timer();
    private final boolean toHub;

    public ShootCommand(ShooterHopper shooter, boolean toHub) {
        this.shooter = shooter;
        this.toHub = toHub;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        //shooter.setUseRealDist(useRealDist);
        timer.reset();
        timer.start();
        shooter.setToHub(toHub);
        shooter.setState(ShooterState.SPEEDING_UP);
    }

    @Override
    public void execute() {
        // log motor speed and iterations of periodic to see how long it takes to spin up
        if (timer.hasElapsed(1.0)) {
            shooter.setState(ShooterState.SHOOTING);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
        // this command should end when the hopper is empty
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        shooter.setState(ShooterState.IDLE);
    }
}
