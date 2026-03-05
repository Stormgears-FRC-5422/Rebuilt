package frc.robot.subsystems.misc;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotState.HopperState;
import frc.robot.sensors.stormnet.StormNet;
import frc.utils.StormSubsystem;
import org.littletonrobotics.junction.Logger;

public class SensorSubsystem extends StormSubsystem {
    private final DigitalInput bottomBeamBreak;

    private StormNet stormNet;
    private RobotState robotState;

    public SensorSubsystem() {
        bottomBeamBreak = new DigitalInput(Constants.SensorSubsystem.bottomBeamBreakPort);

        if (Constants.SensorSubsystem.useStormNet) {
            StormNet.init();
            stormNet = StormNet.getInstance();

            if (Constants.Debug.debug) {
                stormNet.test();
            }
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
    // When StormNet is disabled, falls back to beam break only.
    private HopperState getHopperDepth() {
        boolean bottomBB = !bottomBeamBreak.get();
        Logger.recordOutput("SensorSubsystem/BottomBeamBreak", bottomBB);

        // Beam break: can only distinguish EMPTY from LOW
        HopperState bbState = bottomBB ? HopperState.HOPPER_LOW : HopperState.HOPPER_EMPTY;

        if (stormNet == null) {
            Logger.recordOutput("SensorSubsystem/HopperState", bbState.toString());
            return bbState;
        }

        // Full fusion: lidar (continuous) + beam break (binary)
        double distance = stormNet.getLidarDistance();
        Logger.recordOutput("SensorSubsystem/LidarDistance", distance);

        // Lidar: lower distance = fuller hopper. Negative means bad data.
        HopperState lidarState = HopperState.UNKNOWN;
        if (distance >= 0) {
            if (distance < Constants.SensorSubsystem.lidarDepthFull) {
                lidarState = HopperState.HOPPER_FULL;
            } else if (distance < Constants.SensorSubsystem.lidarDepthHalf) {
                lidarState = HopperState.HOPPER_HALF;
            } else if (distance < Constants.SensorSubsystem.lidarDepthLow) {
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

        Logger.recordOutput("SensorSubsystem/HopperState", result.toString());
        return result;
    }

}
