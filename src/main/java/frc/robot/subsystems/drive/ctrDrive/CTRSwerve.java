package frc.robot.subsystems.drive.ctrDrive;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.RobotState;
import frc.robot.sensors.imu.IMU;
import frc.robot.sensors.imu.IMUFactory;
import frc.robot.sensors.pose.CTRPoseEstimator;
import frc.robot.sensors.pose.PoseEstimator;
import frc.robot.subsystems.drive.DrivetrainBase;
import org.littletonrobotics.junction.Logger;

import java.lang.reflect.Field;
import java.lang.reflect.Method;

import static edu.wpi.first.units.Units.*;

/**
 * CTRE Phoenix 6 swerve drive implementation.
 * Uses CTRE's vendor swerve code with internal IMU and pose estimation.
 * This implementation handles field-relative conversion and driver perspective internally.
 */
public class CTRSwerve extends DrivetrainBase {
    public final CTRDriveInternal driveInternal;
    private final SwerveRequest.FieldCentric driveFieldCentric;
    private final SwerveRequest.RobotCentric driveRobotCentric;
    private final CTRPoseEstimator poseEstimatorWrapper;
    private final IMU wrappedIMU;
    private final Telemetry logger;

    public CTRSwerve(Class<?> tunerConstantsClass) {
        super();

        double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
        double MaxSpeed = getMaxSpeed(tunerConstantsClass);
        driveInternal = getInternalDriveTrain(tunerConstantsClass);

        logger = new Telemetry(MaxSpeed);

        driveFieldCentric = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

        driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

        setMaxVelocities(MetersPerSecond.of(MaxSpeed), RadiansPerSecond.of(MaxAngularRate));

        // Wrap CTRE's internal pose estimation
        poseEstimatorWrapper = new CTRPoseEstimator(driveInternal);

        // Wrap the internal Pigeon2 IMU for external access
        wrappedIMU = IMUFactory.wrap(driveInternal.getPigeon2());

        // Provide driver perspective from base class to internal drivetrain
        driveInternal.setDriverPerspectiveSupplier(this::getDriverPerspective);

        driveInternal.registerTelemetry(logger::telemeterize);
    }

    // ==================== Implementation Characteristics ====================
    // CTRSwerve handles these internally via CTRE's vendor code

    @Override
    protected boolean handlesFieldRelativeConversion() {
        return true;  // CTRE's FieldCentric/RobotCentric requests handle this
    }

    @Override
    protected boolean handlesDriverPerspective() {
        return true;  // CTRDriveInternal handles driver perspective
    }

    @Override
    protected boolean usesInternalIMU() {
        return true;  // CTRE's SwerveDrivetrain has internal Pigeon2
    }

    // ==================== Core API Overrides ====================

    @Override
    public IMU getIMU() {
        return wrappedIMU;
    }

    @Override
    public PoseEstimator getPoseEstimator() {
        return poseEstimatorWrapper;
    }

    // ==================== Orientation Reset ====================

    @Override
    protected void resetOrientationTo(Rotation2d targetRotation) {
        // CTRE-specific: reset pose with new rotation, preserving translation
        Pose2d oldPose = getPose();
        Pose2d newPose = new Pose2d(oldPose.getTranslation(), targetRotation);

        console("Resetting orientation to: " + targetRotation.getDegrees() + " degrees");
        poseEstimatorWrapper.resetPose(newPose);
        state.setPose(newPose);
    }

    // ==================== Drive ====================

    @Override
    public void drive(ChassisSpeeds speeds, boolean fieldRelative, double speedScale) {
        super.drive(speeds, fieldRelative, speedScale);
        console("Drive/DesiredSpeeds" + speeds, 100);
    }

    @Override
    public void periodic() {
        super.periodic();  // Handles resetOrientation option check
        driveInternal.periodic();

        // Log steer motor currents
        SwerveModule[] swerveModules = driveInternal.getModules();
        for (int i = 0; i < swerveModules.length; i++) {
            Logger.recordOutput("steer current " + i, swerveModules[i].getSteerMotor().getTorqueCurrent().getValue());
        }

        if (state.getPeriod() == RobotState.StatePeriod.AUTONOMOUS) {
            console("in CTRDrive chassisSpeeds: " + m_chassisSpeeds, 100);
        }

        // Apply drive control using options.useFieldRelative
        if (m_fieldRelative) {
            driveInternal.setControl(driveFieldCentric
                .withVelocityX(m_chassisSpeeds.vxMetersPerSecond)
                .withVelocityY(m_chassisSpeeds.vyMetersPerSecond)
                .withRotationalRate(m_chassisSpeeds.omegaRadiansPerSecond));
        } else {
            driveInternal.setControl(driveRobotCentric
                .withVelocityX(m_chassisSpeeds.vxMetersPerSecond)
                .withVelocityY(m_chassisSpeeds.vyMetersPerSecond)
                .withRotationalRate(m_chassisSpeeds.omegaRadiansPerSecond));
        }

        // Log module states
        SwerveModuleState[] moduleTargetStates = driveInternal.getState().ModuleTargets;
        SwerveModuleState[] moduleStates = driveInternal.getState().ModuleStates;
        for (int i = 0; i < moduleTargetStates.length; i++) {
            Logger.recordOutput("Drive/Module States " + i, moduleStates[i]);
            Logger.recordOutput("Drive/Module Target " + i, moduleTargetStates[i]);
            Logger.recordOutput("Drive/measuredAngle " + i, MathUtil.angleModulus(moduleStates[i].angle.getRadians()));
        }
    }

    // ==================== Reflection Utilities ====================

    private double getMaxSpeed(Class<?> tunerConstantsClass) {
        try {
            Field field = tunerConstantsClass.getField("kSpeedAt12Volts");
            Object kSpeedAt12Volts = field.get(null); // Static field, so null for instance
            return ((LinearVelocity) kSpeedAt12Volts).in(MetersPerSecond);
        } catch (Exception e) {
            throw new RuntimeException("Failed to access kSpeedAt12Volts: " + e.getMessage(), e);
        }
    }

    private CTRDriveInternal getInternalDriveTrain(Class<?> tunerConstantsClass) {
        try {
            Method method = tunerConstantsClass.getMethod("createDrivetrain");
            return (CTRDriveInternal) method.invoke(null); // Static method, so null for instance
        } catch (Exception e) {
            throw new RuntimeException("Failed to create drivetrain: " + e.getMessage(), e);
        }
    }
}
