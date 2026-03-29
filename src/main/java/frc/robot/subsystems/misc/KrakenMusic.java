package frc.robot.subsystems.misc;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.StormSubsystem;
import frc.robot.Constants.Music;

public class KrakenMusic extends StormSubsystem {
    private final TalonFX motor1;
    private final TalonFX motor2;
    private final Orchestra orchestra = new Orchestra();

    public KrakenMusic() {
        motor1  = new TalonFX(10);
        motor2 = new TalonFX(11);
        orchestra.addInstrument(motor1);
        orchestra.addInstrument(motor2);

        var status = orchestra.loadMusic(Music.file);
        if (!status.isOK()) {
            System.out.println("Failed to load music: " + status.toString());
        }
    }

    @Override
    public void periodic(){
        super.periodic();
        double voltage = RobotController.getBatteryVoltage();
        boolean isEnabled = DriverStation.isEnabled();

        double threshold;
        if (isEnabled) {
            threshold = Music.thresholdEnabled;
        } else {
            threshold = Music.thresholdDisabled;
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

