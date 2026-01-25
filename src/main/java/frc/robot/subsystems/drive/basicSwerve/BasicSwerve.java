package frc.robot.subsystems.drive.basicSwerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Options.DriveOptions;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.sensors.imu.IMU;
import frc.robot.sensors.imu.IMUFactory;

import java.util.Optional;

import frc.robot.subsystems.drive.SimplePoseTelemetry;

import static edu.wpi.first.units.Units.*;

public class BasicSwerve extends DrivetrainBase {

    final SwerveModuleGroup moduleGroup;
    final SwerveModuleBase[] modules;
    final SwerveDriveKinematics kinematics;
    private final SimplePoseTelemetry robotTelemetry;
    private final DriveOptions options;
    private final IMU imu;
    private final SwerveDrivePoseEstimator poseEstimator;
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

        options = DriveOptions.create();

        moduleGroup = new SwerveModuleGroup();
        setMaxVelocities(moduleGroup.getMaxLinearVelocity(),
                moduleGroup.getMaxAngularVelocity());

        kinematics = new SwerveDriveKinematics(moduleGroup.getModuleTranslations());
        modules = moduleGroup.getModules();

        // Initialize pose estimator with starting pose at origin
        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            imu.getRotation(),
            moduleGroup.getModulePositions(),
            new Pose2d()  // Start at origin (0, 0, 0°)
        );
        currentPose = poseEstimator.getEstimatedPosition();

        SmartDashboard.putData("Swerve Drive", builder -> {
            builder.setSmartDashboardType("SwerveDrive");
            builder.addDoubleProperty("Robot Angle", () -> imu.getRotation().getDegrees(), null);
            for (SwerveModuleBase m : modules) {
                builder.addDoubleProperty(m.name + " Angle", () -> m.getSteerAngle().in(Degrees), null);
                builder.addDoubleProperty(m.name + " Velocity", () -> m.getDriveVelocity().in(MetersPerSecond), null);
            }
        });

        // Start simulation thread if in simulation mode
        if (RobotBase.isSimulation()) {
            startSimThread();
        }
    }

    @Override
    public void periodic() {
        super.periodic();

        if (options.resetOrientation.get()) {
            resetOrientation();
            options.resetOrientation.setValue(false);
        }
        moduleGroup.periodic();

        if (options.allowRotation.get()) {
            SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(m_chassisSpeeds);
            setModuleStates(targetStates);
        } else {
            driveWithoutRotation();
        }

        // Update pose estimator with current gyro reading and module positions
        currentPose = poseEstimator.update(
            imu.getRotation(),
            moduleGroup.getModulePositions()
        );

        robotTelemetry.telemeterize(currentPose);
    }

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

    @Override
    public IMU getIMU() {
        return imu;
    }

    @Override
    public Rotation2d getRotation() {
        return imu.getRotation();
    }

    @Override
    public void resetOrientation() {
        // Get alliance from DriverStation (with debug override support)
        Rotation2d targetRotation;

        if (!Constants.Debug.debug || Constants.Debug.defaultAlliance.toLowerCase().equals("auto")) {
            // Use FMS alliance data
            Optional<Alliance> alliance = DriverStation.getAlliance();
            targetRotation = alliance.map(a ->
                a == Alliance.Red ? new Rotation2d(Math.PI) : new Rotation2d()
            ).orElse(new Rotation2d()); // Default to 0° if alliance unknown
        } else {
            // Debug override from RobotState
            RobotState state = RobotState.getInstance();
            targetRotation = state.isAllianceRed()
                ? new Rotation2d(Math.PI)  // 180° for red
                : new Rotation2d();         // 0° for blue
        }

        // Set IMU to target rotation
        imu.setYaw(targetRotation.getDegrees());

        // Reset pose estimator with new orientation
        poseEstimator.resetPosition(
            targetRotation,
            moduleGroup.getModulePositions(),
            new Pose2d()  // Reset to origin with alliance-appropriate heading
        );
    }

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

    public void stop() {
        moduleGroup.stop();
        this.drive(new ChassisSpeeds(), false);
    }
}
