package frc.robot.joysticks;

import frc.utils.joysticks.StormXboxController;

public class ReefscapeXboxController extends ReefscapeJoystick {
    StormXboxController XboxController;

    ReefscapeXboxController(int port) {
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

    @Override
    public boolean getRobotRelative() {
        return XboxController.getLeftTrigger() > 0.2;
    }

    @Override
    public double getTurbo() {
        return XboxController.getRightTrigger();
    }

    @Override
    public boolean zeroGyro() {
        return XboxController.getRightBumperIsPressed();
    }

    @Override
    public boolean coralIntake(){
        return XboxController.getXButtonIsHeld();
    }

    @Override
    public boolean climb() {
        return XboxController.getBButtonIsHeld();
    }

    @Override
    public boolean releaseClimb() {
        return XboxController.getYButtonIsHeld();
    }

    @Override
    public boolean cancelAutoReef() {
        return XboxController.getAButtonIsHeld();
    }

    @Override
    public boolean elevatorTestPid() { return XboxController.getRightLittleButtonIsHeld(); }
}
