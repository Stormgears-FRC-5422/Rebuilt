package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ButtonBoard;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Shooter;
import frc.robot.FieldConstants;
import frc.robot.Options.DriveOptions;
import frc.robot.RobotState;
import frc.robot.joysticks.RebuiltJoystick;
import frc.robot.subsystems.ShooterHopper;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.utils.StormCommand;
import org.littletonrobotics.junction.Logger;

public class MoveWhileShoot extends StormCommand {
    private final DrivetrainBase drivetrain;
    private final ShooterHopper shooterHopper;
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

    public MoveWhileShoot(DrivetrainBase drivetrain, RebuiltJoystick joystick, ShooterHopper shooterHopper) {
        this.drivetrain = drivetrain;
        this.shooterHopper = shooterHopper;
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

        // Cap linear velocity while shooting to prevent misses
        double linearMag = Math.hypot(x, y);
        if (linearMag > Drive.shootMaxSpeedScale) {
            double scale = Drive.shootMaxSpeedScale / linearMag;
            x *= scale;
            y *= scale;
        }

        // PID + feedforward omega to face the hub (continuously updated as robot moves)
        updateGoalAngle();
        double currentAngle = drivetrain.getRotation().getRadians();
        double pidOutput = headingController.calculate(currentAngle, goalAngle);

        // Feedforward: compensate for the goal angle changing as the robot moves
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

        // Telemetry
        double headingError = MathUtil.angleModulus(goalAngle - currentAngle);
        Logger.recordOutput("MoveWhileShoot/HeadingErrorRad", headingError);
        Logger.recordOutput("MoveWhileShoot/GoalAngleRad", goalAngle);
        Logger.recordOutput("MoveWhileShoot/MeasuredAngleRad", currentAngle);
        Logger.recordOutput("MoveWhileShoot/OmegaOutput", omega);
        Logger.recordOutput("MoveWhileShoot/GoalAngleRateRadPerSec", goalAngleRate);
        Logger.recordOutput("MoveWhileShoot/LinearSpeed", Math.hypot(x, y));
        Logger.recordOutput("MoveWhileShoot/DistToHub", RobotState.getInstance().getDistFromHub());
    }

    private void updateGoalAngle() {
        Pose2d hubPose = (RobotState.getInstance().getAlliance() == RobotState.StateAlliance.BLUE)
            ? FieldConstants.hubPositionBLUE() : FieldConstants.hubPositionRED();
        Pose2d current = drivetrain.getPoseEstimator().getPose();

        // Get field-relative velocity for compensation
        ChassisSpeeds fieldVel = drivetrain.getMeasuredFieldSpeeds();
        double vx = fieldVel.vxMetersPerSecond;
        double vy = fieldVel.vyMetersPerSecond;

        // Phase delay compensation: shift pose forward by velocity * phaseDelay
        double px = current.getX() + vx * Shooter.phaseDelaySec;
        double py = current.getY() + vy * Shooter.phaseDelaySec;

       double dist0 = Math.hypot(hubPose.getX() - px, hubPose.getY() - py);
        double tof = shooterHopper.getTimeOfFlight(dist0);
        px = current.getX() + vx * (Shooter.phaseDelaySec + tof);
        py = current.getY() + vy * (Shooter.phaseDelaySec + tof);

        // Iterative TOF lookahead (if single pass proves insufficient)
        // for (int i = 0; i < Shooter.lookaheadIterations; i++) {
        //     double dx = hubPose.getX() - px;
        //     double dy = hubPose.getY() - py;
        //     double dist = Math.hypot(dx, dy);
        //     double tof = shooterHopper.getTimeOfFlight(dist);
        //     px = current.getX() + vx * (Shooter.phaseDelaySec + tof);
        //     py = current.getY() + vy * (Shooter.phaseDelaySec + tof);
        // }

        // Aim from lookahead position to target
        double dx = hubPose.getX() - px;
        double dy = hubPose.getY() - py;
        goalAngle = Math.atan2(dy, dx);

        Logger.recordOutput("MoveWhileShoot/CompensatedDist", Math.hypot(dx, dy));
        Logger.recordOutput("MoveWhileShoot/TOF", shooterHopper.getTimeOfFlight(Math.hypot(dx, dy)));
        Logger.recordOutput("MoveWhileShoot/LookaheadX", px);
        Logger.recordOutput("MoveWhileShoot/LookaheadY", py);
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
