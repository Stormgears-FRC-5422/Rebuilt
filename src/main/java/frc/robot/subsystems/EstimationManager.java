package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.sensors.pose.PoseEstimator;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.utils.StormSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

public class EstimationManager extends StormSubsystem {

    private final DrivetrainBase drivetrain;
    private final PoseEstimator poseEstimator;
    private final VisionSubsystem vision;
    private final RobotState state;

    /** Base vision measurement standard deviations [x(m), y(m), theta(rad)].
     *  theta is set very high because MegaTag2 already uses the gyro for heading.
     *  x/y are scaled dynamically based on tag distance — see computeStdDevs(). */
    private static final double BASE_STD_X = Constants.Vision.stdDevX;
    private static final double BASE_STD_Y = Constants.Vision.stdDevY;
    private static final double BASE_STD_THETA = Constants.Vision.stdDevTheta;
    private static final double SCALE_PIVOT = Constants.Vision.stdDevScalePivot;

    public EstimationManager(DrivetrainBase drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.poseEstimator = drivetrain.getPoseEstimator();
        this.vision = vision;
        this.state = RobotState.getInstance();
    }

    /**
     * Returns the canonical PoseEstimator owned by this manager.
     */
    public PoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    /**
     * Computes distance-scaled vision standard deviations.
     * At the pivot distance (default 2m), stdDevs equal the configured base values.
     * Beyond the pivot, trust decreases with the square of the distance ratio,
     * matching MegaTag2's observed error growth at range.
     */
    private static Matrix<N3, N1> computeStdDevs(double closestTagDist) {
        double ratio = Math.max(closestTagDist, 0.01) / SCALE_PIVOT;
        double scale = ratio * ratio;
        return VecBuilder.fill(BASE_STD_X * scale, BASE_STD_Y * scale, BASE_STD_THETA);
    }

    @Override
    public void periodic() {
        super.periodic();

        // 1. Process odometry samples from drive
        ArrayList<OdometrySample> odometrySamples = drivetrain.getAndClearOdometrySamples();
        for (OdometrySample s : odometrySamples) {
            poseEstimator.updateOdometry(s.timestampSeconds(), s.gyroAngle(), s.modulePositions());
        }

        // 2. Integrate vision measurements
        if (vision != null) {
            ArrayList<VisionMeasurement> measurements = vision.getAndClearMeasurements();
            for (VisionMeasurement m : measurements) {
                if (Constants.Vision.mergeVision) {
                    Pose2d poseBefore = poseEstimator.getPose();

                    Matrix<N3, N1> stdDevs = computeStdDevs(m.closestTagDistanceMeters());
                    poseEstimator.addVisionMeasurement(
                        m.pose(),
                        m.timestampSeconds(),
                        stdDevs
                    );

                    Pose2d poseAfter = poseEstimator.getPose();
                    double correctionMeters = Math.hypot(
                        poseAfter.getX() - poseBefore.getX(),
                        poseAfter.getY() - poseBefore.getY());
                    double correctionDegrees = Math.abs(
                        poseAfter.getRotation().getDegrees() - poseBefore.getRotation().getDegrees());

                    String src = m.sourceLimelight();
                    Logger.recordOutput("Estimation/" + src + "/CorrectionMeters", correctionMeters);
                    Logger.recordOutput("Estimation/" + src + "/CorrectionDegrees", correctionDegrees);
                    Logger.recordOutput("Estimation/" + src + "/PoseBefore", poseBefore);
                    Logger.recordOutput("Estimation/" + src + "/PoseAfter", poseAfter);
                    Logger.recordOutput("Estimation/" + src + "/TagDistMeters", m.closestTagDistanceMeters());
                    Logger.recordOutput("Estimation/" + src + "/StdDevXY", stdDevs.get(0, 0));
                }
            }
        }

        // 3. Push canonical pose to RobotState
        Pose2d canonicalPose = poseEstimator.getPose();
        state.setPose(canonicalPose);
        Logger.recordOutput("Estimation/Pose", canonicalPose);
    }
}
