package frc.robot.subsystems.misc;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;    

public class KrakenMusic extends SubsystemBase {
    private final TalonFX motor1 = new TalonFX(1);
    private final TalonFX motor2 = new TalonFX(2);
    private final Orchestra orchestra = new Orchestra();

    public KrakenMusic() {
        orchestra.addInstrument(motor1);
        orchestra.addInstrument(motor2);

        var status = orchestra.loadMusic("r2d2.chrp");
        if (!status.isOK()) {
            System.out.println("Failed to load music: " + status.toString());
        }
    }

    @Override
    public void periodic(){
        double voltage = RobotController.getBatteryVoltage();
        boolean isEnabled = DriverStation.isEnabled();

        double threshold; 
        if (isEnabled) {
            threshold = 9.0;
        } else {
            threshold = 12.0;
        }

        if (voltage < threshold && !orchestra.isPlaying()) {
            orchestra.play();
        } else if (voltage >= threshold && orchestra.isPlaying()) {
            orchestra.stop();
        }
    }
    public void playR2D2() {
        orchestra.play();
    }

}
