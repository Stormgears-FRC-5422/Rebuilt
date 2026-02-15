package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ShooterHopper;
import frc.robot.subsystems.ShooterHopper.ShooterState;
import frc.utils.StormCommand;

public class ShootCommand extends StormCommand {

    private final ShooterHopper shooter;
    private final Timer timer = new Timer();

    public ShootCommand(ShooterHopper shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        shooter.setState(ShooterState.SPEEDING_UP);
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(0.2)) {
            shooter.setState(ShooterState.SHOOTING);
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(5.0);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter(); 
        timer.stop();
    }
}
