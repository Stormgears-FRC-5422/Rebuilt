package frc.robot.subsystems.drive.ctrDrive;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.lang.reflect.Field;
import java.lang.reflect.Method;

import static edu.wpi.first.units.Units.*;

public class CTRSwerve extends DrivetrainBase {
    public final CTRDriveInternal driveInternal;
    private final SwerveRequest.FieldCentric driveFieldCentric;
    private final SwerveRequest.RobotCentric driveRobotCentric;

    RobotState robotState;
    SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    DoubleArraySubscriber poseSub;
    int count = 0;
    Telemetry logger;

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

        // This class will flip for alliance as necessary, rather than relying on base class
        setDriveFlip(true);
        // This class will handle field-relative conversions, rather than relying on base class
        setFieldRelativeOn(false);

//        drivetrain.resetPose(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)));
        driveInternal.registerTelemetry(logger::telemeterize);
    }

// Reflection functions
    double getMaxSpeed(Class<?> tunerConstantsClass) {
        double maxSpeed = 0;
        try {
            Field field = tunerConstantsClass.getField("kSpeedAt12Volts");
            Object kSpeedAt12Volts = field.get(null); // Static field, so null for instance
            return ((LinearVelocity) kSpeedAt12Volts).in(MetersPerSecond);
        } catch (Exception e) {
            throw new RuntimeException("Failed to access kSpeedAt12Volts: " + e.getMessage(), e);
        }
    }

    CTRDriveInternal getInternalDriveTrain(Class<?> tunerConstantsClass) {
        try {
            Method method = tunerConstantsClass.getMethod("createDrivetrain");
            return (CTRDriveInternal) method.invoke(null); // Static method, so null for instance
        } catch (Exception e) {
            throw new RuntimeException("Failed to create drivetrain: " + e.getMessage(), e);
        }
    }
// end of reflection functions

    @Override
    public void resetOrientation() {
//        // Note that this behavior defaults to blue if alliance is missing.
//        Rotation2d newRotation = m_state.isAllianceRed() ? new Rotation2d(-1, 0) : new Rotation2d(1, 0);
//        console("Reset orientation at degrees: " + newRotation.getDegrees());
//        drivetrain.getPigeon2().setYaw(newRotation.getDegrees());
        Pose2d oldPose = getPose();

        // Note that this behavior defaults to blue if alliance is missing.
        Rotation2d newRotation = state.isAllianceRed()
            ? new Rotation2d(-1, 0)
            : new Rotation2d(1, 0);

        Pose2d newPose = new Pose2d(oldPose.getX(), oldPose.getY(), newRotation);
        declarePoseIsNow(newPose);
    }

    // TODO
    //@Override
    public void declarePoseIsNow(Pose2d newPose) {
        console("declaring pose is now = " + newPose);
        driveInternal.resetPose(newPose);
        state.setPose(newPose);
    }

    // TODO
    // @Override
    @AutoLogOutput
    public Pose2d getPose() {
        Pose2d internalPose = driveInternal.getState().Pose;
        return new Pose2d(internalPose.getTranslation(), internalPose.getRotation());
    }

    @Override
    public void drive(ChassisSpeeds speeds, boolean fieldRelative, double speedScale) {
        super.drive(speeds, fieldRelative, speedScale);
        console("Drive/DesiredSpeeds" + speeds,100);
    }

    @Override
    public void periodic() {
        super.periodic();
        driveInternal.periodic();

        SwerveModule[] swerveModules = driveInternal.getModules();
        for (int i =0; i<swerveModules.length; i++){
            Logger.recordOutput("steer current " + i, swerveModules[i].getSteerMotor().getTorqueCurrent().getValue());
//            Logger.recordOutput("steer current " + i, swerveModules[i].getSteerMotor().getStatorCurrent().getValue());
//            Logger.recordOutput("steer current supply " + i, swerveModules[i].getSteerMotor().getSupplyCurrent().getValue());

//            swerveModules[i].getSteerMotor().setControl(new TorqueCurrentFOC(8));
//            swerveModules[i].getSteerMotor().setControl(new VoltageOut(5));
        }

        if (state.getPeriod() == RobotState.StatePeriod.AUTONOMOUS) {
            console("in CTRDrive chassisSpeeds: " + m_chassisSpeeds, 100);
        }

        if (m_fieldRelative) {
            driveInternal.setControl(driveFieldCentric.withVelocityX(m_chassisSpeeds.vxMetersPerSecond).withVelocityY(m_chassisSpeeds.vyMetersPerSecond).withRotationalRate(m_chassisSpeeds.omegaRadiansPerSecond));
        } else {
            driveInternal.setControl(driveRobotCentric.withVelocityX(m_chassisSpeeds.vxMetersPerSecond).withVelocityY(m_chassisSpeeds.vyMetersPerSecond).withRotationalRate(m_chassisSpeeds.omegaRadiansPerSecond));
        }

        SwerveModuleState[] moduleTargetStates = driveInternal.getState().ModuleTargets;
        SwerveModuleState[] moduleStates = driveInternal.getState().ModuleStates;
        for (int i = 0; i< moduleTargetStates.length; i++){
            Logger.recordOutput("Drive/Module States " + i, moduleStates[i]);
            Logger.recordOutput("Drive/Module Target " + i, moduleTargetStates[i]);
            Logger.recordOutput("Drive/measuredAngle " + i, MathUtil.angleModulus(moduleStates[i].angle.getRadians()));
        }

    }

//    @Override
//    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
//        return driveInternal.sysIdQuasistatic(direction);
//    }
//
//    @Override
//    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
//        return driveInternal.sysIdDynamic(direction);
//    }
}
