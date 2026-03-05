package frc.robot.subsystems.stormnet;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;

// It is intentional that this isn't really a subsystem. Don
public class StormNet {
	private static StormNet instance;
	private Thread m_thread;
	private UDPListener m_listener;

	// Sensors
	private StormNetSensor m_testSensor;
	private MatrixLidar m_lidar;

	private final Map<String, Integer> m_commandMap;
	private int m_commandSize = -1;

	private StormNet() {
		// This MUST match what is coming from the StormNet board itself
		m_commandMap = new HashMap<String, Integer>();
		m_commandMap.put("P", 0); // ping
		m_commandMap.put("F", 1); // fast
		m_commandMap.put("S", 5); // slow
		m_commandMap.put("B", 9); // blink
		m_commandMap.put("M", 13); // lidar
		m_commandMap.put(":", 17); // timer
		m_commandSize = 29; // count of bytes from above

        // TODO Logging

	}

	public static void init() {
		instance = new StormNet();
		instance.connect();
	}

	public static StormNet getInstance() {
		return instance;
	}

	private void connect() {
		m_listener = new UDPListener();
		m_listener.setCommandLocations(m_commandMap, m_commandSize);

		// This might leak a thread if we end up here twice
		// Call stop before connecting a second time
		m_thread = new Thread(m_listener);
		m_thread.start();
		m_testSensor = new StormNetSensor(m_listener);
		m_lidar = new MatrixLidar(m_listener);
	}

	public void stop() {
		m_listener.stop();
	}

	public boolean test() {
		console("In StormNet Test");
        console("about to test core");
        console("waiting for 1 second to make sure we've gotten data");

		try {
			TimeUnit.SECONDS.sleep(1);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

//		m_testSensor.setDebug(true);
//		System.out.println("core test " + (m_testSensor.test() ? "passed" : "failed"));
//		System.out.println("about to test lidar");
//		m_lidar.test();
//		m_testSensor.setDebug(false);
        m_lidar.setDebug(true);
        console("average lidar distance = " + getLidarDistance());
		return true;
	}

	public double getLidarDistance() {
		return m_lidar.getAverageDistanceMM(); // in mm
	}

    public void console(String message) {
        System.out.println("StormNet : " + message);
    }

}
