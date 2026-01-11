package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Options.DriveOptions;
import frc.robot.subsystems.drive.config.SwerveModuleBase;
import frc.robot.subsystems.drive.config.SwerveModuleGroup;
import frc.robot.subsystems.drive.imu.IMU;
import frc.robot.subsystems.drive.imu.IMUFactory;

import static edu.wpi.first.units.Units.*;

public class BasicSwerve extends DrivetrainBase {

    final SwerveModuleGroup moduleGroup;
    final SwerveModuleBase[] modules;
    final SwerveDriveKinematics kinematics;
    private final DriveOptions options;
    private final IMU imu;

    public BasicSwerve() {
        super();

        imu = IMUFactory.createIMU();
        imu.reset();

        options = DriveOptions.create();

        moduleGroup = new SwerveModuleGroup();
        setMaxVelocities(moduleGroup.getMaxLinearVelocity().in(MetersPerSecond),
                moduleGroup.getMaxAngularVelocity().in(RadiansPerSecond));

        kinematics = new SwerveDriveKinematics(moduleGroup.getModuleTranslations());
        modules = moduleGroup.getModules();

        SmartDashboard.putData("Swerve Drive", builder -> {
            builder.setSmartDashboardType("SwerveDrive");
            builder.addDoubleProperty("Robot Angle", () -> -imu.getYaw(), null);
            for (SwerveModuleBase m : modules) {
                builder.addDoubleProperty(m.name + " Angle", m::getSteerAngle, null);
                builder.addDoubleProperty(m.name + " Velocity", m::getDriveVelocity, null);
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
            m.setDriveVelocity(speed);
            m.setSteerAngle(angle);
        }
    }

    @Override
    public Rotation2d getRotation() {
        return imu.getRotation();
    }

    @Override
    public void resetOrientation() {
        imu.reset();
    }

    public void stop() {
        moduleGroup.stop();
        this.drive(new ChassisSpeeds(), false);
    }
}
