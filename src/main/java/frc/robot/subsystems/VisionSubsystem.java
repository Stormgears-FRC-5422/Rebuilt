package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.FieldConstants;
import frc.robot.sensors.pose.PoseEstimator;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.utils.StormSubsystem;
import frc.utils.vision.LimelightHelpers;
import frc.utils.vision.LimelightHelpers.PoseEstimate;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class VisionSubsystem extends StormSubsystem {

    SwerveDrivePoseEstimator poseEstimator;
    String limelightID;

    /** Drivetrain reference — used for gyro heading and pose estimator */
    private final DrivetrainBase drivetrain;
    private final PoseEstimator drivetrainPoseEstimator;

    /** Vision measurement standard deviations [x(m), y(m), theta(rad)].
     *  theta is set very high because MegaTag2 already uses the gyro for heading. */
    private static final Matrix<N3, N1> VISION_STD_DEVS = VecBuilder.fill(0.7, 0.7, 9999999);

    public VisionSubsystem(
//        SwerveDrivePoseEstimator poseEstimator
        String limelightID,
        DrivetrainBase drivetrain
    ){
//        this.poseEstimator = poseEstimator;
        this.limelightID = limelightID;
        this.drivetrain = drivetrain;
        this.drivetrainPoseEstimator = drivetrain.getPoseEstimator();
    }


    public boolean seesTag(){
        double ATid = LimelightHelpers.getFiducialID(limelightID);
        return (ATid >= 1);
    }

    public int getBestTag(){
        return (int) LimelightHelpers.getFiducialID(limelightID);
    }

    public double getDistFromTag(){
        LimelightHelpers.PoseEstimate poseEstimate =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightID);
        Pose2d poseV = poseEstimate.pose;
        Pose2d poseTag = FieldConstants.getPoseTag(getBestTag());
        Translation2d diff = poseV.minus(poseTag).getTranslation();
        double xDist = diff.getX();
        double yDist = diff.getY();
        return Math.sqrt(Math.pow(xDist,2)+ Math.pow(yDist,2));



    }

    public Transform2d getTurretOffset(double xDist1, double pivotDist, int pivotAngle, int planarAngle){
        double pAngle = Math.toRadians(pivotAngle);
        double plAngle = Math.toRadians(planarAngle);

        double plDist = xDist1 + pivotDist * Math.cos(pAngle);

        double xDist = plDist * Math.cos(plAngle);
        double yDist = plDist * Math.sin(plAngle);

        return new Transform2d(xDist,yDist, new Rotation2d(planarAngle));
    }



    @Override
    public void periodic() {
        super.periodic();

        // --- Pose fusion: feed MegaTag2 vision pose into drivetrain estimator ---

        // MegaTag2 requires the robot orientation to be set each cycle
        double robotYaw = drivetrain.getRotation().getDegrees();
        LimelightHelpers.SetRobotOrientation(limelightID, robotYaw, 0, 0, 0, 0, 0);

        // Retrieve the MegaTag2 pose estimate (blue-origin field coordinates)
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightID);

        // Reject bad measurements
        if (estimate == null || estimate.tagCount == 0) {
            Logger.recordOutput("Vision/HasTarget", false);
            return;
        }

        Logger.recordOutput("Vision/HasTarget", true);
        Logger.recordOutput("Vision/TagCount", estimate.tagCount);
        Logger.recordOutput("Vision/Pose", estimate.pose);
        Logger.recordOutput("Vision/AvgTagDist", estimate.avgTagDist);

        // Merge the vision measurement into the drivetrain's pose estimator
        drivetrainPoseEstimator.addVisionMeasurement(
            estimate.pose,
            estimate.timestampSeconds,
            VISION_STD_DEVS
        );
    }
}
