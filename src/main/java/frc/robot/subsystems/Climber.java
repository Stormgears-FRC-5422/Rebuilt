package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoeix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static java.lang.System.console;

public class Climber extends SubsystemBase {
    // MOTORS
    private final TalonFX leadMotor;
    private final TalonFX followerMotor;

    public enum ClimberState {
        UNKNOWN,
        IDLE,
        HOMING,
        READYTOCLIMB,
        CLIMBING,
        CLIMBED,
        MANUALCLIMB,
    }
    public ClimberState state = ClimberState.UNKNOWN;
    private final TalonFXConfiguration config = new TalonFXConfiguration();
    private double leadMotorSpeed = 0.0;

    public enum Direction {
        FORWARD,
        BACKWARD,
    }



    /**
     * constructs a new climber system & initializes motors
     *
     *
     */

    public Climber() {
        leadMotor = new TalonFX(53, "");
        followerMotor = new TalonFX(54, "");
        followerMotor.setControl(
            new Follower(leadMotor.getDeviceID(), MotorAlignmentValue.Opposed)
        );
        config.Feedback.SensorToMechanismRatio = 16;// .Opposed
    }

    public void setState(ClimberState newState) {
        state = newState;

        switch (state) {
            case UNKNOWN -> {
                break;
            }
            case HOMING -> {
                break;
            }
            case IDLE -> {
                break;
            }
            case READYTOCLIMB ->{
                break;
            }
            case CLIMBING -> {
                break;
            }
            case CLIMBED -> {
                break;
            }
            case MANUALCLIMB -> {
                break;
            }
        }

    }


    /**
     * This method is called periodically by the scheduler.
     */

    @Override
    public void periodic() {
        super.periodic();
        switch (state) {
            case UNKNOWN -> {
                leadMotorSpeed = 0;
                break;
            }
            case HOMING -> {
                home();
                break;
            }
            case IDLE -> {
                break;
            }
            case READYTOCLIMB -> {
                break;
            }
            case CLIMBING -> {
                break;
            }
            case CLIMBED -> {
                break;
            }
            case MANUALCLIMB -> {
                break;
            }
        }
        leadMotor.set(leadMotorSpeed);

    }

    /**
     * This method is called periodically by the scheduler.
     */

    public void setClimberState(Climber.ClimberState state) {

    }

    private void home() {
        if (Constants.Shooter.stratlimit) {
            // might want to use an average here to minimize spikes
            leadMotor.setVoltage(-2.0);
        } else { //assume you're at home
            leadMotor.setVoltage(0.0);
        }
    }

    public boolean isAtHome() {
        if (Constants.Shooter.stratlimit) {
            // might want to use an average here to minimize spikes
            return (leadMotor.getStatorCurrent().getValueAsDouble()) > Constants.Shooter.intakeCurrentLimit;
        } else {
            return true;
        }
    }

}
