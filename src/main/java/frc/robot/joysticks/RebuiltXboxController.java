package frc.robot.joysticks;

import frc.utils.joysticks.StormXboxController;

public class RebuiltXboxController extends RebuiltJoystick {
    private final StormXboxController xboxController;

    RebuiltXboxController(int port) {
        super(new StormXboxController(port));
        this.xboxController = (StormXboxController) driveJoystick;
    }

    // Core driving inherited from base class via DriveJoystick

    @Override
    public boolean getIntakeRun() {
        return xboxController.getLeftTrigger() > 0.2;
    }

    @Override
    public double getTurbo() {
        return xboxController.getRightTrigger();
    }

    @Override
    public boolean resetGyro() {
        return xboxController.getRightBumperIsPressed();
    }

    @Override
    public boolean narrow() {
        return xboxController.getYButtonIsHeld();
    }
    @Override
    public boolean autoShoot() {
        return xboxController.getAButtonIsHeld();
    }

    @Override
    public boolean intakeHome() {
        return xboxController.getLeftBumperIsHeld();
    }

    // getSlider() uses base default (0.0)
    // @Override
    // public double getLeftTriggerAxis() {
    //     return xboxController.getLeftTrigger();
    // }
    @Override
    public boolean getRobotRelative() {
        return xboxController.getLeftBumperIsPressed();
    }
    @Override
    public boolean intakeTogglePressed() {
        return xboxController.getAButtonIsPressed();
    }
    @Override
    public boolean enableRumble() {
        return xboxController.getXisPressed();
    }
    @Override
    public boolean shootToAlliance() {
        return xboxController.getYButtonIsPressed();
    }

    @Override
    public boolean manualShoot() {
        return xboxController.getBButtonIsHeld();
    }

    @Override
    public boolean intakeDown() {
        return xboxController.getXButtonIsHeld();
    }
}
