package frc.robot.subsystems.drive.basicSwerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Options.DriveOptions;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.sensors.imu.IMU;
import frc.robot.sensors.imu.IMUFactory;

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
        imu.reset();
        // Reset pose to origin with current field-relative heading
        poseEstimator.resetPosition(
            imu.getRotation(),
            moduleGroup.getModulePositions(),
            new Pose2d()
        );
    }

    public void stop() {
        moduleGroup.stop();
        this.drive(new ChassisSpeeds(), false);
    }
}
