package frc.robot.joysticks;

import frc.utils.joysticks.StormXboxController;

public class RebuiltXboxController extends RebuiltJoystick {
    StormXboxController XboxController;

    RebuiltXboxController(int port) {
        super(port);
        XboxController = new StormXboxController(port);
    }

    @Override
    public double getWpiX() {
        return XboxController.getWpiXSpeed();
    }

    @Override
    public double getWpiY() {
        return XboxController.getWpiYSpeed();
    }

    @Override
    public double getOmegaSpeed() {
        return XboxController.getOmegaSpeed();
    }

    //@Override
    public boolean getRobotRelative() {
        return XboxController.getLeftTrigger() > 0.2;
    }

    //@Override
    public double getTurbo() {
        return XboxController.getRightTrigger();
    }

    //@Override
    public boolean zeroGyro() {
        return XboxController.getRightBumperIsPressed();
    }

    //@Override
    public boolean wheelRadiusCharacterization() {
        return XboxController.getYButtonIsHeld();
    }
}
