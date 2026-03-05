package frc.robot.sensors.stormnet;

import frc.robot.Constants;

public class MatrixLidar extends StormNetSensor {
	private final long[] sensorDetails;
	private int metricCount;

	public MatrixLidar(StormNetVoice voice) {
		super(voice);

		// two bytes per sensor
        // [0] is the distance in mm from sensor #0, [1] is sensor quality indicator, [2] starts sensor #1
        metricCount = Constants.StormNet.lidarMetricCount;
		sensorDetails = new long[metricCount];
		this.m_deviceString = voice.getDeviceString();
	}

	public void pollDetails() {
		fetchLongs("M", "MatricLidarValues", sensorDetails);
	}

	// Note that -1 means no good data
    public double getAverageDistanceMM() {
        pollDetails();
        double distance;;

		// Measurements are in mm, we want cm
		distance = (double) sensorDetails[0];

		return distance;
    }


}
