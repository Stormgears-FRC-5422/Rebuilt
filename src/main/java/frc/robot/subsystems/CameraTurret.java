package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;

public class CameraTurret {
    // two servos
    private Servo cameraPan;
    private Servo cameraTilt;

    // values; take in an angle between 0 and 180
    private int tilt;
    private int pan;

    public CameraTurret() {
        cameraPan = new Servo(0);
        cameraTilt = new Servo(1);
        tilt = 0;
        pan = 0;
    }
    
    public void periodic() {
        // pan is 0 -> 180
        // tilt is 0 -> 90, normalize  value
        if (tilt > 90) tilt = 90;
        setTilt();
        setPan();
    }
    public void setTilt() {
        cameraTilt.setAngle(tilt);
    }
    public void setPan() {
        cameraPan.setAngle(pan);
    }
    public void end() {
        pan = 0;
        tilt = 0;
    }
}
