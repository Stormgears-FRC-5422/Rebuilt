package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.utils.StormSubsystem;
import frc.robot.RobotState;

public class BeamBreakSensor extends StormSubsystem{

    private final DigitalInput bottomBeamBreak;
    private final DigitalInput topBeamBreak;
    private final RobotState m_robotState;

    public BeamBreakSensor() {

        bottomBeamBreak = new DigitalInput(8); // place holder channels
        topBeamBreak = new DigitalInput(7); // place holder channels

        m_robotState = RobotState.getInstance();
    }
    public boolean isHopperHalf(){
        return bottomBeamBreak.get();
    }
    public boolean isHopperFull(){
        return topBeamBreak.get();
    }
    @Override
    public void periodic() {
        super.periodic();

        boolean bottom = isHopperHalf();
        boolean top = isHopperFull();
        RobotState.HopperState state;

        if(!top && !bottom){ // hopper half
            state = RobotState.HopperState.HOPPER_EMPTY;
        }
        else if(isHopperFull() == true && isHopperHalf() == true){ // hopper full
            state = RobotState.HopperState.HOPPER_FULL;
        }
        else if(isHopperFull() == false && isHopperHalf() == false){ // hopper empty
            state = RobotState.HopperState.HOPPER_FULL;
        }
        else{
            state = RobotState.HopperState.UNKNOWN;
        }

        m_robotState.setHopperState(state);
    }
}
