package frc.robot.joysticks;

public class ReefscapeJoystickFactory {

    public static ReefscapeJoystick getInstance(String joystickType, int port) throws IllegalJoystickTypeException {
        ReefscapeJoystick instance;
        System.out.println("Initializing " + joystickType + " as Joystick");
        switch (joystickType.toLowerCase()) {
            case "xboxcontroller" -> instance = new ReefscapeXboxController(port);
            case "buttonboard" -> instance = new ReefscapeButtonBoard(port);
            default -> throw new IllegalJoystickTypeException("Illegal Joystick Type: " + joystickType + " ---!");
        }
        return instance;
    }
}
