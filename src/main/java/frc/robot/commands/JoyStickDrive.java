package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.ButtonBoard;
import frc.robot.Constants.Drive;
import frc.robot.Options.DriveOptions;
import frc.robot.joysticks.RebuiltJoystick;
import frc.robot.joysticks.RebuiltLogitechJoystick;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.utils.StormCommand;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static java.util.Objects.isNull;

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
        addRequirements(drivetrain);

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

        if (turboSupplier.getAsDouble() <= 0.2) {
            drivetrain.setDriveSpeedScale(Drive.precisionSpeedScale);
        } else {
            drivetrain.setDriveSpeedScale(turboSupplier.getAsDouble());
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

        ChassisSpeeds speeds = new ChassisSpeeds(x, y, omega);
        drivetrain.percentOutputDrive(speeds, fieldRelativeSupplier.getAsBoolean());

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
