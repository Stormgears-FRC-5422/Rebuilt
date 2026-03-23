package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.FieldConstants;
import frc.robot.Constants.Drive;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.utils.StormCommand;

public class FaceTowardHub extends StormCommand {
    private final PIDController headingController;

    DrivetrainBase drivetrain;
    double goalAngle;

    public FaceTowardHub(DrivetrainBase drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        headingController = new PIDController(Drive.targetKPvcmd, Drive.targetKIvcmd, Drive.targetKDvcmd);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        headingController.setTolerance(Math.toRadians(Drive.targetToleranceDegrees));
    }

    @Override
    public void initialize() {
        super.initialize();

        //Calculate Translation distance between Hub and current Robot Loc.
        Pose2d hubPose = (RobotState.getInstance().getAlliance() == RobotState.StateAlliance.BLUE) ? FieldConstants.hubPositionBLUE() : FieldConstants.hubPositionRED();
        Pose2d current = RobotState.getInstance().getPose();
        Translation2d trans = hubPose.getTranslation().minus(current.getTranslation());

        //Finds goal angle based on X and Y component of the translation
        goalAngle = trans.getAngle().getRadians();

        headingController.reset();
        headingController.setSetpoint(goalAngle);
    }

    @Override
    public void execute() {
        super.execute();

        //Uses PID to calc optimal rotation speed
        double currentAngle = drivetrain.getRotation().getRadians();
        double omega = headingController.calculate(currentAngle, goalAngle);
        drivetrain.drive(new ChassisSpeeds(0, 0, omega), false);
    }

    @Override
    public boolean isFinished() {
        return headingController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0, 0, 0), false);
        super.end(interrupted);
    }
}
