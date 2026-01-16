package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.drive.basicSwerve.BasicSwerve;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.utils.StormCommand;

public class HoldSwerveModuleStates extends StormCommand {

    private final BasicSwerve swerveDrive;
    private final LinearVelocity speed;
    private final Angle angle;
    private SwerveModuleState[] targetStates;

    public HoldSwerveModuleStates(DrivetrainBase drive, LinearVelocity speed, Angle angle) {
        if (!(drive instanceof BasicSwerve)) {
            throw new IllegalArgumentException("HoldSwerveModuleStates command requires a BasicSwerve drivetrain");
        }
        safeAddRequirements(drive);

        this.swerveDrive = (BasicSwerve) drive;
        this.speed = speed;
        this.angle = angle;
    }

    @Override
    public void initialize() {
        super.initialize();

        targetStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            targetStates[i] = new SwerveModuleState(speed, new Rotation2d(angle));
        }
    }

    @Override
    public void execute() {
        super.execute();
        swerveDrive.setTargetModuleStates(targetStates);
    }

    @Override
    public boolean isFinished() {
        return false; // Run until interrupted
    }

    @Override
    public void end(boolean interrupted) {
        // Stop all modules but maintain their angles
        swerveDrive.stop();
        super.end(interrupted);
    }
}
