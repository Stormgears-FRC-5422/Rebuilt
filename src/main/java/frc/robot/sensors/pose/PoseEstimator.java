package frc.robot.sensors.pose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Interface for pose estimation operations.
 * Abstracts pose estimation for different hardware/implementation approaches.
 * EstimationManager owns the update cycle — drive implementations produce
 * odometry data, EstimationManager feeds it to the estimator.
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

    /**
     * Update the estimator with an odometry sample.
     * Default no-op for implementations that handle odometry internally (e.g., CTRE).
     */
    default void updateOdometry(double timestampSeconds, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        // no-op
    }
}
