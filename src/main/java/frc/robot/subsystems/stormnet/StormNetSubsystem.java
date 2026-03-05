package frc.robot.subsystems.stormnet;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotState.HopperState;

import frc.utils.StormSubsystem;
import org.littletonrobotics.junction.Logger;



public class StormNetSubsystem extends StormSubsystem {
    private final DigitalInput bottomBeamBreak;
    // private final DigitalInput topBeamBreak;

    private StormNet stormNet;
    private RobotState robotState;

    public StormNetSubsystem() {
        bottomBeamBreak = new DigitalInput(Constants.StormNet.bottomBeamBreakPort);
        // topBeamBreak = new DigitalInput(Constants.StormNet.topBeamBreakPort);

        StormNet.init();
        stormNet = StormNet.getInstance();

        if (Constants.Debug.debug) {
            stormNet.test();
        }

        robotState = RobotState.getInstance();
    }

    @Override
    public void periodic() {
        super.periodic();

        robotState.setHopperState(getHopperDepth());
    }

    // Report hopper depth based on all available sensors.
    // Combines lidar (continuous distance) with beam break (binary).
    // Takes the "fuller" of the two readings as the conservative estimate.
    private HopperState getHopperDepth() {
        double distance = stormNet.getLidarDistance();
        boolean bottomBB = bottomBeamBreak.get();
        // boolean topBB = topBeamBreak.get();

        Logger.recordOutput("StormNet/LidarDistance", distance);
        Logger.recordOutput("StormNet/BottomBeamBreak", bottomBB);

        // Beam break: can only distinguish EMPTY from LOW
        HopperState bbState = bottomBB ? HopperState.HOPPER_LOW : HopperState.HOPPER_EMPTY;

        // Lidar: lower distance = fuller hopper. Negative means bad data.
        HopperState lidarState = HopperState.UNKNOWN;
        if (distance >= 0) {
            if (distance < Constants.StormNet.lidarDepthFull) {
                lidarState = HopperState.HOPPER_FULL;
            } else if (distance < Constants.StormNet.lidarDepthHalf) {
                lidarState = HopperState.HOPPER_HALF;
            } else if (distance < Constants.StormNet.lidarDepthLow) {
                lidarState = HopperState.HOPPER_LOW;
            } else {
                lidarState = HopperState.HOPPER_EMPTY;
            }
        }

        // Combine: if lidar data is bad, trust beam break alone.
        // Otherwise take whichever says more full (conservative).
        // Beam break can only report EMPTY or LOW, so the only interesting case
        // is lidar=EMPTY but beam break=LOW.
        HopperState result;
        if (lidarState == HopperState.UNKNOWN) {
            result = bbState;
        } else if (lidarState == HopperState.HOPPER_EMPTY && bbState == HopperState.HOPPER_LOW) {
            result = bbState;
        } else {
            result = lidarState;
        }

        Logger.recordOutput("StormNet/HopperState", result.toString());
        return result;
    }

}
