package frc.robot.subsystems.drive.basicSwerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.OdometrySample;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.drive.SimplePoseTelemetry;
import frc.robot.sensors.imu.IMU;
import frc.robot.sensors.imu.IMUFactory;
import frc.robot.sensors.pose.PoseEstimator;
import frc.robot.sensors.pose.WPILibPoseEstimator;

import static edu.wpi.first.units.Units.*;

/**
 * Custom swerve drive implementation using WPILib's kinematics and pose estimation.
 * Supports both SparkMax (NEO) and TalonFX (Kraken) modules via SwerveModuleFactory.
 */
public class BasicSwerve extends DrivetrainBase {

    private final SwerveModuleGroup moduleGroup;
    private final SwerveModuleBase[] modules;
    private final SwerveDriveKinematics kinematics;
    private final SimplePoseTelemetry robotTelemetry;
    private final IMU imu;
    private final SwerveDrivePoseEstimator internalPoseEstimator;
    private final WPILibPoseEstimator poseEstimatorWrapper;
    private Pose2d currentPose;

    // Simulation support
    private static final double kSimLoopPeriod = 0.005; // 5ms = 200 Hz
    private Notifier simNotifier = null;
    private double lastSimTime;

    public BasicSwerve() {
        super();
        robotTelemetry = new SimplePoseTelemetry("robotPose");

        imu = IMUFactory.createIMU();
        imu.reset();

        moduleGroup = new SwerveModuleGroup();
        setMaxVelocities(moduleGroup.getMaxLinearVelocity(),
                moduleGroup.getMaxAngularVelocity());

        kinematics = new SwerveDriveKinematics(moduleGroup.getModuleTranslations());
        modules = moduleGroup.getModules();

        // Initialize pose estimator with starting pose at origin
        internalPoseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            imu.getRotation(),
            moduleGroup.getModulePositions(),
            new Pose2d()  // Start at origin (0, 0, 0°)
        );

        // Wrap in our PoseEstimator interface
        poseEstimatorWrapper = new WPILibPoseEstimator(
            internalPoseEstimator,
            imu::getRotation,
            moduleGroup::getModulePositions
        );

        currentPose = internalPoseEstimator.getEstimatedPosition();

        // Start simulation thread if in simulation mode
        if (RobotBase.isSimulation()) {
            startSimThread();
        }
    }

    @Override
    public void periodic() {
        super.periodic();  // Handles resetOrientation option check

        moduleGroup.periodic();

        if (options.allowRotation.get()) {
            SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(m_chassisSpeeds);
            setModuleStates(targetStates);
        } else {
            driveWithoutRotation();
        }

        // Queue odometry for EstimationManager to process
        addOdometrySample(new OdometrySample(
            edu.wpi.first.wpilibj.Timer.getFPGATimestamp(),
            imu.getRotation(),
            moduleGroup.getModulePositions()
        ));

        currentPose = poseEstimatorWrapper.getPose();
        robotTelemetry.telemeterize(currentPose);
    }

    // ==================== Core API Overrides ====================

    @Override
    public IMU getIMU() {
        return imu;
    }

    @Override
    public PoseEstimator getPoseEstimator() {
        return poseEstimatorWrapper;
    }

    @Override
    protected String[] getDashboardModuleNames() {
        String[] names = new String[modules.length];
        for (int i = 0; i < modules.length; i++) {
            names[i] = modules[i].name;
        }
        return names;
    }

    @Override
    protected SwerveModuleState[] getDashboardModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = new SwerveModuleState(
                modules[i].getDriveVelocity().in(MetersPerSecond),
                Rotation2d.fromDegrees(modules[i].getSteerAngle().in(Degrees)));
        }
        return states;
    }

    // ==================== Module Control ====================

    public void setModuleStates(SwerveModuleState[] states) {
        moduleGroup.setSwerveModuleStates(states);
    }

    public void setTargetModuleStates(SwerveModuleState[] states) {
        m_chassisSpeeds = kinematics.toChassisSpeeds(states);
    }

    private void driveWithoutRotation() {
        LinearVelocity speed = MetersPerSecond.of(
                Math.hypot(m_chassisSpeeds.vxMetersPerSecond, m_chassisSpeeds.vyMetersPerSecond));
        Angle angle = Radians.of(
                Math.atan2(m_chassisSpeeds.vyMetersPerSecond, m_chassisSpeeds.vxMetersPerSecond));

        for (SwerveModuleBase m : modules) {
            m.setTargets(speed, angle);
        }
    }

    public void stop() {
        moduleGroup.stop();
        this.drive(new ChassisSpeeds(), false);
    }

    // ==================== Simulation ====================

    private void startSimThread() {
        lastSimTime = Timer.getFPGATimestamp();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = new Notifier(() -> {
            final double currentTime = Timer.getFPGATimestamp();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            /* Use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(kSimLoopPeriod);
    }

    private void updateSimState(double deltaTime, double batteryVoltage) {
        // Update module simulations
        moduleGroup.updateSimState(deltaTime, batteryVoltage);

        // Update IMU simulation based on robot rotation
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(moduleGroup.getModuleStates());
        double angularVelocityRadPerSec = speeds.omegaRadiansPerSecond;

        // Simulate gyro rotation (IMUBase provides default no-op implementation)
        imu.updateSimulation(angularVelocityRadPerSec * deltaTime);
    }
}
