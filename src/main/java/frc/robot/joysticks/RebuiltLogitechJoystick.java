package frc.robot.joysticks;

import frc.utils.joysticks.StormLogitechController;

public class RebuiltLogitechJoystick extends RebuiltJoystick {
    private final StormLogitechController controller;

    public RebuiltLogitechJoystick(int port) {
        super(new StormLogitechController(port));
        this.controller = (StormLogitechController) driveJoystick;
    }

    // Core driving inherited from base class via DriveJoystick

    @Override
    public double getSlider() {
        return controller.getSliderAxis();
    }

    @Override
    public boolean testAutoShoot() {
        return controller.getPOV() == 0; // D-pad up
    }
}
