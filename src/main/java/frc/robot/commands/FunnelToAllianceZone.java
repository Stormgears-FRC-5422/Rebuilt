package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ButtonBoard;
import frc.robot.Constants.Drive;
import frc.robot.Options.DriveOptions;
import frc.robot.joysticks.RebuiltJoystick;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.utils.StormCommand;
import org.littletonrobotics.junction.Logger;

public class FunnelToAllianceZone extends StormCommand {
    private final DrivetrainBase drivetrain;
    private final Pose2d targetPose;
    private final DoubleSupplier txSupplier;
    private final DoubleSupplier tySupplier;
    private final DoubleSupplier turboSupplier;
    private final BooleanSupplier fieldRelativeSupplier;

    private final PIDController headingController;
    private SlewRateLimiter xScaleLimiter;
    private SlewRateLimiter yScaleLimiter;
    private DriveOptions options;

    double goalAngle;
    private double previousGoalAngle;
    private double previousTimestamp;

    public FunnelToAllianceZone(DrivetrainBase drivetrain, RebuiltJoystick joystick, Pose2d targetPose) {
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;
        addRequirements(drivetrain);

        txSupplier = joystick::getWpiX;
        tySupplier = joystick::getWpiY;
        turboSupplier = joystick::getTurbo;

        options = DriveOptions.create();
        fieldRelativeSupplier = () -> options.useFieldRelative.get() && !joystick.getRobotRelative();

        headingController = new PIDController(Drive.targetKPvcmd, Drive.targetKIvcmd, Drive.targetKDvcmd);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        headingController.setTolerance(Math.toRadians(Drive.targetToleranceDegrees));
    }

    @Override
    public void initialize() {
        super.initialize();
        drivetrain.setDriveSpeedScale(Drive.driveSpeedScale);
        xScaleLimiter = new SlewRateLimiter(Drive.linearRateLimiter);
        yScaleLimiter = new SlewRateLimiter(Drive.linearRateLimiter);
        headingController.reset();
        updateGoalAngle();
        previousGoalAngle = goalAngle;
        previousTimestamp = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        super.execute();

        if (turboSupplier.getAsDouble() <= 0.2) {
            drivetrain.setDriveSpeedScale(Drive.precisionSpeedScale);
        } else {
            drivetrain.setDriveSpeedScale(turboSupplier.getAsDouble());
        }

        double x = txSupplier.getAsDouble();
        double y = tySupplier.getAsDouble();

        if (ButtonBoard.squarePath) {
            x = xScaleLimiter.calculate(x * Math.abs(x));
            y = yScaleLimiter.calculate(y * Math.abs(y));
        } else {
            x = xScaleLimiter.calculate(x);
            y = yScaleLimiter.calculate(y);
        }

        double linearMag = Math.hypot(x, y);
        if (linearMag > Drive.shootMaxSpeedScale) {
            double scale = Drive.shootMaxSpeedScale / linearMag;
            x *= scale;
            y *= scale;
        }

        updateGoalAngle();
        double currentAngle = drivetrain.getRotation().getRadians();
        double pidOutput = headingController.calculate(currentAngle, goalAngle);

        double now = Timer.getFPGATimestamp();
        double dt = now - previousTimestamp;
        double goalAngleRate = 0;
        if (dt > 0 && dt < 0.1) {
            goalAngleRate = MathUtil.angleModulus(goalAngle - previousGoalAngle) / dt;
        }
        previousGoalAngle = goalAngle;
        previousTimestamp = now;

        double omega = MathUtil.clamp(pidOutput + Drive.targetKFvcmd * goalAngleRate, -1.0, 1.0);

        boolean fieldRelative = fieldRelativeSupplier.getAsBoolean();
        ChassisSpeeds speeds = new ChassisSpeeds(x, y, omega);

        if (fieldRelative) {
            Rotation2d perspective = drivetrain.getDriverPerspective();
            if (perspective != null) {
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, perspective.unaryMinus());
            }
        }

        drivetrain.percentOutputDrive(speeds, fieldRelative);

        double headingError = MathUtil.angleModulus(goalAngle - currentAngle);
        Logger.recordOutput("FunnelToAllianceZone/HeadingErrorRad", headingError);
        Logger.recordOutput("FunnelToAllianceZone/GoalAngleRad", goalAngle);
        Logger.recordOutput("FunnelToAllianceZone/MeasuredAngleRad", currentAngle);
        Logger.recordOutput("FunnelToAllianceZone/OmegaOutput", omega);
        Logger.recordOutput("FunnelToAllianceZone/GoalAngleRateRadPerSec", goalAngleRate);
        Logger.recordOutput("FunnelToAllianceZone/LinearSpeed", Math.hypot(x, y));
    }

    private void updateGoalAngle() {
        Pose2d current = drivetrain.getPoseEstimator().getPose();
        Translation2d trans = targetPose.getTranslation().minus(current.getTranslation());
        goalAngle = Math.atan2(trans.getY(), trans.getX());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0, 0, 0), false);
        super.end(interrupted);
    }
}
