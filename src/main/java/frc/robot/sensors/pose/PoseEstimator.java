package frc.robot.sensors.pose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Interface for pose estimation operations.
 * Abstracts pose estimation for different hardware/implementation approaches.
 * This is a thin wrapper - implementations do NOT manage the update cycle.
 * Each drive implementation keeps its own update logic in periodic().
 */
public interface PoseEstimator {
    /**
     * Get the current estimated pose of the robot.
     * @return Current pose estimate
     */
    Pose2d getPose();

    /**
     * Reset the pose estimator to a specific pose.
     * @param pose The new pose
     */
    void resetPose(Pose2d pose);

    /**
     * Reset only the rotation while preserving translation.
     * @param rotation The new rotation
     */
    void resetRotation(Rotation2d rotation);

    /**
     * Add a vision measurement to the pose estimator.
     * @param visionPose The measured pose from vision
     * @param timestampSeconds The timestamp of the measurement
     * @param stdDevs Standard deviations for the measurement [x, y, theta]
     */
    void addVisionMeasurement(Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs);
}
