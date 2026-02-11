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
    String[] limelightIDs;
    String limelightID;

    /** Drivetrain reference — used for gyro heading and pose estimator */
    private final DrivetrainBase drivetrain;
    private final PoseEstimator drivetrainPoseEstimator;

    /** Maximum distance to accept a tag measurement, in feet */
    private static final double MAX_TAG_DISTANCE_FEET = 9.0;
    private static final double MAX_TAG_DISTANCE_METERS = MAX_TAG_DISTANCE_FEET * 0.3048;

    /** Vision measurement standard deviations [x(m), y(m), theta(rad)].
     *  theta is set very high because MegaTag2 already uses the gyro for heading. */
    private static final Matrix<N3, N1> VISION_STD_DEVS = VecBuilder.fill(0.7, 0.7, 9999999);

    public VisionSubsystem(
//        SwerveDrivePoseEstimator poseEstimator,
        DrivetrainBase drivetrain,
        String... limelightIDs
    ){
//        this.poseEstimator = poseEstimator;
        this.limelightIDs = limelightIDs;
        this.drivetrain = drivetrain;
        this.drivetrainPoseEstimator = drivetrain.getPoseEstimator();
    }


    public boolean seesTag(String limelightID) {
        double ATid = LimelightHelpers.getFiducialID(limelightID);
        return (ATid >= 1);
    }

    public int getBestTag(String limelightID) {
        return (int) LimelightHelpers.getFiducialID(limelightID);
    }

    public double getDistFromTag(String limelightID) {
        LimelightHelpers.PoseEstimate poseEstimate =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightID);
        if (poseEstimate == null || poseEstimate.tagCount == 0) {
            return Double.MAX_VALUE;
        }
        return poseEstimate.avgTagDist;
    }

    public Transform2d getTurretOffset(double xDist1, double pivotDist, int pivotAngle, int planarAngle) {
        double pAngle = Math.toRadians(pivotAngle);
        double plAngle = Math.toRadians(planarAngle);

        double plDist = xDist1 + pivotDist * Math.cos(pAngle);

        double xDist = plDist * Math.cos(plAngle);
        double yDist = plDist * Math.sin(plAngle);

        return new Transform2d(xDist, yDist, new Rotation2d(planarAngle));
    }

    public String getClosestLL() {
        String closestLL = null;
        double minDist = Double.MAX_VALUE;

        for (String ll : limelightIDs) {
            if (seesTag(ll)) {
                double dist = getDistFromTag(ll);
                if (dist < minDist) {
                    minDist = dist;
                    closestLL = ll;
                }
            }
        }

        // If no limelight sees a tag, return the first one as fallback
        if (closestLL == null && limelightIDs.length > 0) {
            closestLL = limelightIDs[0];
        }

        return closestLL;
    }

    public void setAllLimelightOrientations(double robotYaw) {
        for (String ll : limelightIDs) {
            LimelightHelpers.SetRobotOrientation(ll, robotYaw, 0, 0, 0, 0, 0);
        }
    }

    @Override
    public void periodic() {
        super.periodic();

        // MegaTag2 requires the robot orientation to be set each cycle for all limelights
        double robotYaw = drivetrain.getRotation().getDegrees();
        setAllLimelightOrientations(robotYaw);

        limelightID = getClosestLL();

        // Retrieve the MegaTag2 pose estimate (blue-origin field coordinates)
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightID);

        // Reject bad measurements
        if (estimate == null || estimate.tagCount == 0) {
            Logger.recordOutput("Vision/HasTarget", false);
            return;
        }

        // Reject tags that are too far away (unreliable at distance)
        if (estimate.avgTagDist > MAX_TAG_DISTANCE_METERS) {
            Logger.recordOutput("Vision/HasTarget", false);
            Logger.recordOutput("Vision/RejectedDist", estimate.avgTagDist);
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
