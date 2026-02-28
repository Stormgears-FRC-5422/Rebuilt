package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.utils.StormCommand;

public class AlignNarrow extends StormCommand {
    private final DrivetrainBase drivetrain;
    private final PIDController headingController;

    private static final double goalanglerad = 0.0;

    public AlignNarrow(DrivetrainBase drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        headingController = new PIDController(
            Drive.targetKPvcmd,
            Drive.targetKIvcmd,
            Drive.targetKDvcmd
        );
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        headingController.setTolerance(Math.toRadians(Drive.targetToleranceDegrees));
    }

    @Override
    public void initialize() {
        super.initialize();
        headingController.reset();
        headingController.setSetpoint(goalanglerad);
    }

    @Override
    public void execute() {
        super.execute();

        double currentAngle = drivetrain.getRotation().getRadians();
        double omega = headingController.calculate(currentAngle, goalanglerad);

        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, omega), false);
    }

    @Override
    public boolean isFinished() {
        return headingController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0), false);
        super.end(interrupted);
    }
}

