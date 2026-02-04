package frc.robot.commands;
import frc.utils.StormCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class SetShooterVelocity extends StormCommand {
    private final ShooterSubsystem shooter;
    private final double velocity;

    public SetShooterVelocity(ShooterSubsystem shooter, double velocity) {
        this.shooter = shooter;
        this.velocity = velocity;
        addRequirements(shooter);
    }
    @Override
    public void initialize() {
        shooter.setState(ShooterSubsystem.ShooterState.SPEEDING_UP);
    }
    @Override
    public void execute() {
        shooter.setTargetVelocity(velocity);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter(); //state changes r handled in subsystem
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
