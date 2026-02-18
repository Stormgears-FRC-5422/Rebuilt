package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.utils.StormCommand;

public class FaceTowardHub extends StormCommand {

    DrivetrainBase drivetrain;
    double goalAngle;
    private final PIDController headingController = new PIDController(3.1, 0.0, 0.1);


    public FaceTowardHub(DrivetrainBase drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        headingController.setTolerance(Math.toRadians(2));
    }

    @Override
    public void initialize() {
        //Calculate Translation distance between Hub and current Robot Loc.
        Pose2d hubPose = ((RobotState.getInstance().getAlliance() == RobotState.StateAlliance.BLUE) ? FieldConstants.hubPositionBLUE() : FieldConstants.hubPositionRED());
        Pose2d current = drivetrain.getPoseEstimator().getPose();
        Translation2d trans = hubPose.getTranslation().minus(current.getTranslation());

        //Finds goal angle based on X and Y component of the transltion
        double y = trans.getY();
        double x = trans.getX();
        goalAngle = Math.atan2(y, x);

        headingController.reset();
        headingController.setSetpoint(goalAngle);

        super.initialize();
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
