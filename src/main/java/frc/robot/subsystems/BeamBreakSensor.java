package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.utils.StormSubsystem;
import frc.robot.RobotState;

public class BeamBreakSensor extends StormSubsystem{

    private enum HopperState{
        hopperFull,
        hopperHalf,
        hopperEmpty,
        unknown,
    }
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
        if(isHopperFull() == false && isHopperHalf() == true){ // hopper half
            // hopperState = HopperState.hopperHalf;
            m_robotState.setHopperState(m_robotState.hopperState.hopperHalf);
        }
        else if(isHopperFull() == true && isHopperHalf() == true){ // hopper full
            // hopperState = HopperState.hopperFull;
            m_robotState.setHopperState(m_robotState.hopperState.hopperFull);
        }
        else if(isHopperFull() == false && isHopperHalf() == false){ // hopper empty
            // hopperState = HopperState.hopperEmpty;
            m_robotState.setHopperState(m_robotState.hopperState.hopperEmpty);
        }
    }
}
