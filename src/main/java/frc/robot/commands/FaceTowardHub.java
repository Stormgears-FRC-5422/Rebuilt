package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.utils.StormCommand;

public class FaceTowardHub extends StormCommand {

    DrivetrainBase drivetrain;
    double goalAngle;
    public FaceTowardHub(DrivetrainBase drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        Pose2d hubPose = ((RobotState.getInstance().getAlliance() == RobotState.StateAlliance.BLUE)? FieldConstants.hubPositionBLUE(): FieldConstants.hubPositionRED());
        Pose2d current = drivetrain.getPoseEstimator().getPose();
        Translation2d trans = current.minus(hubPose).getTranslation();

        double x = Math.abs(trans.getX());
        double y = Math.abs(trans.getY());
        goalAngle = Math.atan(x/y);


        super.initialize();
    }

    @Override
    public void execute() {
        if(drivetrain.getPoseEstimator().getPose().getRotation().getRadians() != goalAngle){
            drivetrain.drive(new ChassisSpeeds(0,0, 0.5), false);
        }

    }

}
