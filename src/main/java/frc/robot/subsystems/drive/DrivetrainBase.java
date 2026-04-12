package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Drive;
import frc.robot.Options.DriveOptions;
import frc.robot.RobotState;
import frc.robot.sensors.imu.IMU;
import frc.robot.sensors.imu.NullIMU;
import frc.robot.sensors.pose.NullPoseEstimator;
import frc.robot.sensors.pose.PoseEstimator;
import frc.robot.subsystems.OdometrySample;
import frc.utils.StormSubsystem;

import java.util.ArrayList;

/**
 * Abstract base class for all drivetrain implementations.
 * Provides common functionality for field-relative driving, pose estimation,
 * and orientation management.
 */
public abstract class DrivetrainBase extends StormSubsystem {
    protected final RobotState state;
    protected final DriveOptions options;

    protected LinearVelocity m_maxVelocity = MetersPerSecond.of(1);
    protected AngularVelocity m_maxAngularVelocity = RadiansPerSecond.of(1);
    protected double m_driveSpeedScale = 0;

    /** Stores whether the current drive command is field-relative (for child classes that handle conversion) */
    protected boolean m_fieldRelative = false;

    /** Cached driver perspective - only updated when disabled */
    private Rotation2d m_cachedDriverPerspective = null;
    private boolean m_hasAppliedDriverPerspective = false;

    private final PIDController xController = new PIDController(Drive.trajectoryXKP, Drive.trajectoryXKI, Drive.trajectoryXKD);
    private final PIDController yController = new PIDController(Drive.trajectoryYKP, Drive.trajectoryYKI, Drive.trajectoryYKD);
    private final PIDController headingController = new PIDController(Drive.trajectoryHeadingKP, Drive.trajectoryHeadingKI, Drive.trajectoryHeadingKD);

    @AutoLogOutput
    protected ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    /** Odometry samples collected during periodic(), consumed by EstimationManager */
    private ArrayList<OdometrySample> pendingOdometrySamples = new ArrayList<>();

    /**
     * Store an odometry sample for EstimationManager to process.
     * Called by drive implementations instead of updating the estimator directly.
     */
    protected void addOdometrySample(OdometrySample sample) {
        pendingOdometrySamples.add(sample);
    }

    /**
     * Returns all pending odometry samples and clears the list.
     * Called by EstimationManager each cycle.
     */
    public ArrayList<OdometrySample> getAndClearOdometrySamples() {
        ArrayList<OdometrySample> samples = pendingOdometrySamples;
        pendingOdometrySamples = new ArrayList<>();
        return samples;
    }

    /** Collision manager for simulation (null on real robot or if disabled) */
    private frc.robot.simulation.collision.CollisionManager collisionManager;

    private boolean m_dashboardRegistered = false;
    private final SimplePoseTelemetry robotTelemetry = new SimplePoseTelemetry("robotPose");

    public DrivetrainBase() {
        state = RobotState.getInstance();
        options = DriveOptions.create();
        setDriveSpeedScale(Drive.driveSpeedScale);
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        // Initialize collision detection for simulation only
        collisionManager = frc.robot.simulation.collision.CollisionManager.createIfEnabled();
    }

    // ==================== Implementation Characteristics ====================
    // These describe how the implementation is structured (NOT user-selectable options)

    /**
     * Returns true if the child handles field-relative conversion internally.
     * When true, the base drive() method passes speeds through unchanged.
     * Default: false (base class handles field-relative conversion)
     */
    protected boolean handlesFieldRelativeConversion() {
        return false;
    }

    /**
     * Returns true if the child handles driver perspective/alliance flip internally.
     * When true, the base class does not apply driver perspective adjustments.
     * Default: false (base class handles driver perspective)
     */
    protected boolean handlesDriverPerspective() {
        return false;
    }

    /**
     * Get the driver perspective angle for field-relative driving.
     * Returns the cached value that is only updated when disabled.
     * This ensures driving behavior doesn't change mid-match.
     *
     * @return The driver perspective as a Rotation2d
     */
    public Rotation2d getDriverPerspective() {
        // Return cached value if available, otherwise parse fresh (for first call before periodic)
        if (m_cachedDriverPerspective != null) {
            return m_cachedDriverPerspective;
        }
        return parseDriverPerspective();
    }

    /**
     * Parse the driver perspective from options.
     * If driverPerspective option is "auto", returns alliance-based default (0° blue, 180° red).
     * Otherwise, parses the option as degrees and returns that angle.
     */
    private Rotation2d parseDriverPerspective() {
        String configured = options.driverPerspective.get();
        if (configured == null || configured.equalsIgnoreCase("auto")) {
            // Default: alliance-based perspective (only if alliance is known)
            if (state.getAlliance() == RobotState.StateAlliance.MISSING) {
                return null;
            }
            return state.isAllianceRed() ? Rotation2d.k180deg : Rotation2d.kZero;
        }
        try {
            double angle = Double.parseDouble(configured);
            return Rotation2d.fromDegrees(angle);
        } catch (NumberFormatException e) {
            // Invalid value, fall back to alliance-based
            console("Invalid driverPerspective value: " + configured + ", using auto");
            if (state.getAlliance() == RobotState.StateAlliance.MISSING) {
                return null;
            }
            return state.isAllianceRed() ? Rotation2d.k180deg : Rotation2d.kZero;
        }
    }

    /**
     * Returns true if the child manages its own IMU internally.
     * When true, the child may still expose a wrapped IMU via getIMU().
     * Default: false (child provides IMU directly)
     */
    protected boolean usesInternalIMU() {
        return false;
    }

    /**
     * Returns the ordered module names for the SmartDashboard SwerveDrive widget.
     * Override in concrete drivetrain subclasses (e.g., "Front Left", "Front Right", ...).
     * Default: no modules (widget shows robot angle only).
     */
    protected String[] getDashboardModuleNames() {
        return new String[0];
    }

    /**
     * Returns the current module states for the SmartDashboard SwerveDrive widget.
     * Must match the order of getDashboardModuleNames().
     * Default: no modules.
     */
    protected SwerveModuleState[] getDashboardModuleStates() {
        return new SwerveModuleState[0];
    }

    // ==================== Core API ====================

    /**
     * Get the IMU used by this drivetrain.
     * Subclasses should override to return their actual IMU (or wrapped internal IMU).
     * @return IMU instance (NullIMU if not overridden)
     */
    public IMU getIMU() {
        return new NullIMU();
    }

    /**
     * Get the pose estimator used by this drivetrain.
     * Subclasses should override to return their actual pose estimator.
     * @return PoseEstimator instance (NullPoseEstimator if not overridden)
     */
    public PoseEstimator getPoseEstimator() {
        return new NullPoseEstimator();
    }

    /**
     * Get the current robot rotation from the IMU.
     * @return Current rotation
     */
    public Rotation2d getRotation() {
        return getIMU().getRotation();
    }

    /**
     * Get the measured field-relative chassis speeds.
     * Converts robot-relative measured speeds to field-relative using current rotation.
     * @return Field-relative ChassisSpeeds
     */
    public ChassisSpeeds getMeasuredFieldSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(m_chassisSpeeds, getRotation());
    }

    /**
     * Get the current estimated pose of the robot.
     * @return Current pose estimate
     */
    public Pose2d getPose() {
        return getPoseEstimator().getPose();
    }

    /**
     * Set the robot's pose to a specific value.
     * Also sets the IMU yaw to match the pose rotation.
     * @param pose The new pose
     */
    public void setPose(Pose2d pose) {
        getIMU().setYaw(pose.getRotation().getDegrees());
        getPoseEstimator().resetPose(pose);
    }

    // ==================== Velocity Configuration ====================
    protected final void setMaxVelocities(LinearVelocity maxVelocity, AngularVelocity maxAngularVelocity) {
        m_maxVelocity = maxVelocity;
        m_maxAngularVelocity = maxAngularVelocity;
        console("MaxDriveVelocity: " + m_maxVelocity.in(MetersPerSecond));
        console("MaxAngularVelocity: " + m_maxAngularVelocity.in(RadiansPerSecond));
    }

    public final void setDriveSpeedScale(double scale) {
        m_driveSpeedScale = MathUtil.clamp(scale, 0, Drive.driveSpeedScale);
    }

    /**
     * Scale chassis speeds uniformly. X and Y must be scaled the same
     * or the robot will move in the wrong direction!
     */
    public ChassisSpeeds scaleChassisSpeeds(ChassisSpeeds speeds, double scale) {
        return new ChassisSpeeds(
            scale * speeds.vxMetersPerSecond,
            scale * speeds.vyMetersPerSecond,
            scale * speeds.omegaRadiansPerSecond
        );
    }

    // ==================== Drive Methods ====================

    /**
     * Command the robot to drive. This method expects real speeds in meters/second.
     * Speed may be limited by speedScale.
     *
     * @param speeds Chassis speeds
     * @param fieldRelative True for field relative driving
     */
    public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
        drive(speeds, fieldRelative, m_driveSpeedScale);
    }

    /**
     * Command the robot to drive with explicit speed scale.
     * The fieldRelative parameter comes from the caller (e.g., joystick command reading options.useFieldRelative).
     * The base class checks whether it should handle the conversion or pass it through to the child.
     *
     * @param speeds Chassis speeds
     * @param fieldRelative True for field relative driving
     * @param speedScale Speed scale factor
     */
    public void drive(ChassisSpeeds speeds, boolean fieldRelative, double speedScale) {
        // Store fieldRelative for child classes that handle conversion themselves
        m_fieldRelative = fieldRelative;

        if (!handlesFieldRelativeConversion() && fieldRelative) {
            // Convert from field frame to robot frame using current gyro rotation.
            // Driver perspective is applied upstream (JoyStickDrive) so that
            // autonomous commands (followTrajectory) use raw field coordinates unmodified.
            m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRotation());
        } else {
            // Either not field-relative, or child handles conversion
            m_chassisSpeeds = speeds;
        }
        m_chassisSpeeds = scaleChassisSpeeds(m_chassisSpeeds, speedScale);
    }

    /**
     * Command the robot to drive using percent output (-1 to 1).
     * Scales inputs to the configured max speeds.
     *
     * @param speeds Chassis speeds as percent output (-1 to 1)
     * @param fieldRelative True for field relative driving
     */

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement.
     * Subclasses should override with swerve-specific implementation.
     */
    public void stopWithX() {
        drive(new ChassisSpeeds(0, 0, 0), false);
    }

    public void percentOutputDrive(ChassisSpeeds speeds, boolean fieldRelative) {
        drive(
            new ChassisSpeeds(
                speeds.vxMetersPerSecond * m_maxVelocity.in(MetersPerSecond),
                speeds.vyMetersPerSecond * m_maxVelocity.in(MetersPerSecond),
                speeds.omegaRadiansPerSecond * m_maxAngularVelocity.in(RadiansPerSecond)
            ),
            fieldRelative
        );
    }

    // ==================== Orientation Reset ====================

    /**
     * Reset orientation to face the correct direction for the current alliance.
     * Uses RobotState for alliance detection (handles both FMS and debug modes).
     *
     * For blue alliance: 0 degrees (facing red wall)
     * For red alliance: 180 degrees (facing blue wall)
     */
    public void resetOrientation() {
        // Get target rotation based on alliance from RobotState
        Rotation2d targetRotation = state.isAllianceRed()
            ? Rotation2d.k180deg  // 180 degrees for red
            : Rotation2d.kZero;   // 0 degrees for blue

        resetOrientationTo(targetRotation);
    }

    /**
     * Reset orientation to a specific target rotation.
     * Subclasses can override for implementation-specific behavior.
     *
     * @param targetRotation The target rotation to reset to
     */
    protected void resetOrientationTo(Rotation2d targetRotation) {
        // Set IMU yaw and reset pose estimator rotation
        getIMU().setYaw(targetRotation.getDegrees());

        Pose2d currentPose = getPose();
        getPoseEstimator().resetPose(new Pose2d(
            currentPose.getTranslation(),
            targetRotation
        ));
    }

    public abstract double getYawRateDegPerSec();

    public void seedTurnEncoder(int moduleIndex) {
        // Override in subclass if supported
    }

    private BooleanSupplier isShootingSupplier = () -> false;

    public void setShootingSupplier(BooleanSupplier supplier) {
        isShootingSupplier = supplier;
    }

    protected boolean isShooting() {
        return isShootingSupplier.getAsBoolean();
    }

    protected boolean isChassisSpeedsZero() {
        return
               Math.abs(m_chassisSpeeds.vxMetersPerSecond) < Drive.xLockVelocity
            && Math.abs(m_chassisSpeeds.vyMetersPerSecond) < Drive.xLockVelocity
            && Math.abs(m_chassisSpeeds.omegaRadiansPerSecond) < Drive.xLockOmega;
    }

    // ==================== Periodic ====================

    @Override
    public void periodic() {
        super.periodic();

        // Register the SwerveDrive widget on the first periodic call, not in the constructor.
        // addDoubleProperty eagerly calls getters when NT is connected, which would fire during
        // the super() chain before subclass field initializers have run.
        if (!m_dashboardRegistered) {
            m_dashboardRegistered = true;
            SmartDashboard.putData("Swerve Drive", builder -> {
                builder.setSmartDashboardType("SwerveDrive");
                builder.addDoubleProperty("Robot Angle", () -> getRotation().getDegrees(), null);
                String[] names = getDashboardModuleNames();
                for (int i = 0; i < names.length; i++) {
                    final int idx = i;
                    builder.addDoubleProperty(names[idx] + " Angle",
                        () -> getDashboardModuleStates()[idx].angle.getDegrees(), null);
                    builder.addDoubleProperty(names[idx] + " Velocity",
                        () -> getDashboardModuleStates()[idx].speedMetersPerSecond, null);
                }
            });
        }

        // Check resetOrientation option (centralized here)
        if (options.resetOrientation.get()) {
            resetOrientation();
            options.resetOrientation.setValue(false);
        }

        // Update driver perspective cache only when disabled.
        // This ensures driving behavior doesn't change until an explicit disable event.
        // JoyStickDrive reads getDriverPerspective() to apply perspective before calling drive().
        if (!m_hasAppliedDriverPerspective || edu.wpi.first.wpilibj.DriverStation.isDisabled()) {
            m_cachedDriverPerspective = parseDriverPerspective();
            if (m_cachedDriverPerspective != null) {
                m_hasAppliedDriverPerspective = true;
            }
        }

        // Collision constraint (simulation only)
        if (collisionManager != null) {
            collisionManager.updateAndConstrain(getPose(), this::setPose);
        }

        // Telemetry only — EstimationManager owns state.setPose()
        robotTelemetry.telemeterize(getPoseEstimator().getPose());
    }

    // ==================== PID / etc Characterization ====================
    public double[] getWheelRadiusCharacterizationPositions() {
        return new double[] {};
    }

    public void followTrajectory(SwerveSample sample) {
        Pose2d pose = getPose();

        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
        );
        drive(speeds, true, 1.0);  // Always full speed: Choreo velocities are already in m/s
    }

}
