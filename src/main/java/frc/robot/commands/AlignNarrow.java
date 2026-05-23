package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.ButtonBoard;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Toggles;
import frc.robot.Options.DriveOptions;
import frc.robot.joysticks.RebuiltJoystick;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.utils.StormCommand;

public class AlignNarrow extends StormCommand {
    private final DrivetrainBase drivetrain;
    private final DoubleSupplier txSupplier;
    private final DoubleSupplier tySupplier;
    private final DoubleSupplier turboSupplier;
    private final BooleanSupplier fieldRelativeSupplier;

    private final PIDController headingController;
    private SlewRateLimiter xScaleLimiter;
    private SlewRateLimiter yScaleLimiter;
    private final DriveOptions options;

    private static final double goalanglerad  = 0.0;

    public AlignNarrow(DrivetrainBase drivetrain, RebuiltJoystick joystick) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        txSupplier = joystick::getWpiX;
        tySupplier = joystick::getWpiY;
        turboSupplier = joystick::getTurbo;

        options = DriveOptions.create();
        fieldRelativeSupplier = () -> options.useFieldRelative.get() && !joystick.getRobotRelative();

        headingController = new PIDController(
            Drive.targetKPvcmd,
            Drive.targetKIvcmd,
            Drive.targetKDvcmd
        );
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
        headingController.setSetpoint(goalanglerad );
    }

    @Override
    public void execute() {
        super.execute();

        if (Drive.useTurbo) {
            if (turboSupplier.getAsDouble() <= 0.2) {
                drivetrain.setDriveSpeedScale(Drive.precisionSpeedScale);
            } else {
                drivetrain.setDriveSpeedScale(turboSupplier.getAsDouble());
            }
        } else {
            double stickMag = Math.hypot(txSupplier.getAsDouble(), tySupplier.getAsDouble());
            drivetrain.setDriveSpeedScale(Drive.minDriveSpeedScale + stickMag * (1.0 - Drive.minDriveSpeedScale));
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

        double currentAngle = drivetrain.getRotation().getRadians();
        double omega = headingController.calculate(currentAngle, goalanglerad );

        omega = MathUtil.clamp(omega, -1.0, 1.0);

        boolean fieldRelative = fieldRelativeSupplier.getAsBoolean();
        ChassisSpeeds speeds = new ChassisSpeeds(x, y, omega);

        if (fieldRelative) {
            Rotation2d perspective = drivetrain.getDriverPerspective();
            if (perspective != null) {
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, perspective.unaryMinus());
            }
        }

        drivetrain.percentOutputDrive(speeds, fieldRelative);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0), false); //todo grab the stop drive function idk whereit is
        super.end(interrupted);
    }
}