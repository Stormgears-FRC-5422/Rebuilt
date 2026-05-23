package frc.robot.commands;

import static java.util.Objects.isNull;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

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

public class JoyStickDrive extends StormCommand {
    private final DrivetrainBase drivetrain;
    private final DoubleSupplier txSupplier;
    private final DoubleSupplier tySupplier;
    private final DoubleSupplier omegaSupplier;
    private final BooleanSupplier fieldRelativeSupplier;
    private final DoubleSupplier turboSupplier;

    private SlewRateLimiter xScaleLimiter;
    private SlewRateLimiter yScaleLimiter;
    private SlewRateLimiter omegaScaleLimiter;

    private DriveOptions options;

    // Note that we want the joystick to mostly work even if there is no drive.
    // So this won't really do anything if the drivetrain is NULL, but it will log.
    public JoyStickDrive(DrivetrainBase drivetrain,
                         RebuiltJoystick joystick) {
        if (!isNull(drivetrain)) {
            console("adding drivetrain requirements");
            addRequirements(drivetrain);
        } else {
            console("supplied drivetrain is NULL");
        }

        this.drivetrain = drivetrain;

        txSupplier = joystick::getWpiX;
        tySupplier = joystick::getWpiY;
        omegaSupplier = joystick::getOmegaSpeed;
        turboSupplier = joystick::getTurbo;

        options = DriveOptions.create();

        // Field-relative when option is enabled AND trigger is NOT pressed
        // Trigger pressed = robot-relative override
        fieldRelativeSupplier = () -> options.useFieldRelative.get() && !joystick.getRobotRelative();
    }

    @Override
    public void initialize() {
        super.initialize();
        drivetrain.setDriveSpeedScale(Drive.driveSpeedScale);
        xScaleLimiter = new SlewRateLimiter(Drive.linearRateLimiter);
        yScaleLimiter = new SlewRateLimiter(Drive.linearRateLimiter);
        omegaScaleLimiter = new SlewRateLimiter(Drive.turnRateLimiter);
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
        double omega = omegaSupplier.getAsDouble();

        if (ButtonBoard.squarePath) {
            x = xScaleLimiter.calculate(x * Math.abs(x));
            y = yScaleLimiter.calculate(y * Math.abs(y));
            omega = omegaScaleLimiter.calculate(omega * Math.abs(omega));
        } else {
            x = xScaleLimiter.calculate(x);
            y = yScaleLimiter.calculate(y);
            omega = omegaScaleLimiter.calculate(omega);
        }

        boolean fieldRelative = fieldRelativeSupplier.getAsBoolean();
        ChassisSpeeds speeds = new ChassisSpeeds(x, y, omega);

        // Apply driver perspective: rotate from driver frame to field frame.
        // drive() then converts field→robot via gyro. Autonomous commands bypass
        // this (they call drive() directly with raw field coordinates).
        if (fieldRelative) {
            Rotation2d perspective = drivetrain.getDriverPerspective();
            if (perspective != null) {
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, perspective.unaryMinus());
            }
        }

        drivetrain.percentOutputDrive(speeds, fieldRelative);

        Logger.recordOutput("x", x);
        Logger.recordOutput("y", y);
        Logger.recordOutput("joy stick speeds", speeds);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
