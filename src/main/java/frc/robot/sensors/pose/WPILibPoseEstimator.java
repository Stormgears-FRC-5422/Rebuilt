package frc.robot.sensors.pose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.util.function.Supplier;

/**
 * Wraps WPILib's SwerveDrivePoseEstimator for use with the PoseEstimator interface.
 * This is a thin wrapper - the update cycle is managed by the drive subsystem.
 */
public class WPILibPoseEstimator implements PoseEstimator {
    private final SwerveDrivePoseEstimator estimator;
    private final Supplier<Rotation2d> gyroSupplier;
    private final Supplier<SwerveModulePosition[]> modulePositionSupplier;

    /**
     * Creates a wrapper around an existing SwerveDrivePoseEstimator.
     * @param estimator The underlying WPILib pose estimator
     * @param gyroSupplier Supplier for current gyro rotation (needed for resetPose)
     * @param modulePositionSupplier Supplier for current module positions (needed for resetPose)
     */
    public WPILibPoseEstimator(
            SwerveDrivePoseEstimator estimator,
            Supplier<Rotation2d> gyroSupplier,
            Supplier<SwerveModulePosition[]> modulePositionSupplier) {
        this.estimator = estimator;
        this.gyroSupplier = gyroSupplier;
        this.modulePositionSupplier = modulePositionSupplier;
    }

    @Override
    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    @Override
    public void resetPose(Pose2d pose) {
        estimator.resetPosition(
            gyroSupplier.get(),
            modulePositionSupplier.get(),
            pose
        );
    }

    @Override
    public void resetRotation(Rotation2d rotation) {
        Pose2d currentPose = getPose();
        resetPose(new Pose2d(currentPose.getTranslation(), rotation));
    }

    @Override
    public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        estimator.addVisionMeasurement(visionPose, timestampSeconds, stdDevs);
    }

    /**
     * Get the underlying SwerveDrivePoseEstimator for direct access if needed.
     * Use this for operations not covered by the interface (e.g., update()).
     * @return The underlying estimator
     */
    public SwerveDrivePoseEstimator getUnderlying() {
        return estimator;
    }
}
