package frc.utils.configfile;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.HashMap;
import java.util.Properties;
import java.util.function.Function;

public class StormProp {

    private static final String PATH = Filesystem.getDeployDirectory().getPath();
    private static final String CONFIG_NAME = "config.properties";
    private static final String BACKUP_NAME = "config_backup.properties";
    private static final File CONFIG_FILE = new File(PATH, CONFIG_NAME);

    private static final String RC_PATH = "/home/lvuser";
    private static final String RC_NAME = ".stormrc";
    private static final File RC_FILE = new File(RC_PATH, RC_NAME);

    private static final HashMap<String, Double> NUMBER_MAP = new HashMap<>();
    private static final HashMap<String, Integer> INTEGER_MAP = new HashMap<>();
    private static final HashMap<String, Boolean> BOOLEAN_MAP = new HashMap<>();
    private static final HashMap<String, String> STRING_MAP = new HashMap<>();
    private static Properties properties;
    private static Properties overrideProperties;
    private static Properties simProperties;
    private static boolean initialized = false;
    private static boolean overrideInitialized = false;
    private static boolean simInitialized = false;
    private static boolean debug = false;

    public static void init() {
        System.out.println("Running in directory " + System.getProperty("user.dir"));
        System.out.println("Trying to use file " + CONFIG_FILE.getAbsolutePath());
        properties = new Properties();
        FileInputStream inputStream = null;
        try {
            inputStream = new FileInputStream(CONFIG_FILE);
            properties.load(inputStream);
            System.out.println("*****************");
            System.out.println("Loading Properties");
            System.out.println("*****************");
        } catch (FileNotFoundException e) {
            System.out.println("Using backup config file");
            try {
                inputStream = new FileInputStream(new File("/home/lvuser/deploy", BACKUP_NAME));
                properties.load(inputStream);
            } catch (FileNotFoundException fnf) {
                System.out.println("Failed to find backup file: " + fnf.getMessage());
            } catch (IOException ioe) {
                System.out.println("Error reading backup file: " + ioe.getMessage());
            }
        } catch (IOException e) {
            System.out.println("Error reading config file: " + e.getMessage());
        } finally {
            if (inputStream != null) {
                try {
                    inputStream.close();
                } catch (IOException e) {
                    System.out.println("Error closing config file: " + e.getMessage());
                }
            }
        }
        initialized = true;
        if (!overrideInitialized) {
            overrideInit();
        }

        debug = getBoolean("debugProperties", false);
    }

    public static void overrideInit() {
        String overrideName = RobotBase.isSimulation()
                ? properties.getProperty("simOverride")
                : properties.getProperty("override");

        overrideName = removeCast(overrideName);

        if (overrideName.equalsIgnoreCase("auto")) {
            System.out.println("Using AUTOMATIC configuration");
            Properties rcProperties = new Properties();
            FileInputStream rcStream = null;
            try {
                System.out.println("rcFile path: " + RC_FILE.getAbsolutePath());
                rcStream = new FileInputStream(RC_FILE);
                rcProperties.load(rcStream);
                System.out.println("autoConfig setting " + rcProperties.getProperty("autoConfig"));
                overrideName = properties.getProperty(rcProperties.getProperty("autoConfig"));
            } catch (FileNotFoundException e) {
                System.out.println("Failed to find autoConfig file: " + e.getMessage());
            } catch (IOException e) {
                System.out.println("Error reading autoConfig file: " + e.getMessage());
            } finally {
                if (rcStream != null) {
                    try {
                        rcStream.close();
                    } catch (IOException e) {
                        System.out.println("Error closing autoConfig file: " + e.getMessage());
                    }
                }
            }
        }

        System.out.println("Using override file " + overrideName);
        File overrideConfigFile = new File(PATH, overrideName);
        overrideProperties = new Properties();

        FileInputStream overrideInputStream = null;
        try {
            overrideInputStream = new FileInputStream(overrideConfigFile);
            overrideProperties.load(overrideInputStream);
        } catch (FileNotFoundException e) {
            System.out.println("!!! No override file detected !!!");
        } catch (IOException e) {
            System.out.println("Error reading override file: " + e.getMessage());
        } finally {
            if (overrideInputStream != null) {
                try {
                    overrideInputStream.close();
                } catch (IOException e) {
                    System.out.println("Error closing override file: " + e.getMessage());
                }
            }
        }

        overrideInitialized = true;
    }

    public static void simInit() {
        simProperties = new Properties();

        // Ignore the sim overrides file unless we are in simulation!
        if (!RobotBase.isSimulation()) {
            simInitialized = true;
            return;
        }

        String simName = properties.getProperty("simOverrideOverride");
        simName = removeCast(simName);

        System.out.println("Using simulation override file " + simName);
        File simConfigFile = new File(PATH, simName);

        FileInputStream simInputStream = null;
        try {
            simInputStream = new FileInputStream(simConfigFile);
            simProperties.load(simInputStream);
        } catch (FileNotFoundException e) {
            System.out.println("!!! No simulation override file detected !!!");
        } catch (IOException e) {
            System.out.println("Error reading simulation file: " + e.getMessage());
        } finally {
            if (simInputStream != null) {
                try {
                    simInputStream.close();
                } catch (IOException e) {
                    System.out.println("Error closing simulation file: " + e.getMessage());
                }
            }
        }

        simInitialized = true;
    }

    private static String removeCast(String value) {
        if (value == null) {
            throw new IllegalArgumentException("Property value cannot be null");
        }
        int endIndex = value.indexOf(")");
        if (endIndex == -1) {
            throw new IllegalArgumentException("Malformed property value, missing type cast: " + value);
        }
        return value.substring(endIndex + 1).trim();
    }

    private static String getPropString(String key) {
        ensureInitialized();

        if (simProperties != null && simProperties.containsKey(key)) {
            return removeCast(simProperties.getProperty(key));
        }
        if (overrideProperties != null && overrideProperties.containsKey(key)) {
            return removeCast(overrideProperties.getProperty(key));
        }
        if (properties != null && properties.containsKey(key)) {
            return removeCast(properties.getProperty(key));
        }
        return "";
    }

    private static void ensureInitialized() {
        if (!initialized) {
            init();
        }
        if (!overrideInitialized) {
            overrideInit();
        }
        if (!simInitialized) {
            simInit();
        }
    }

    private static <T> T getValueWithCache(String key, String prefix, T defaultVal,
            HashMap<String, T> cache,
            Function<String, T> parser) {
        String fullKey = prefix.equals("general") ? key : prefix + "." + key;

        try {
            if (cache.containsKey(fullKey)) {
                return cache.get(fullKey);
            }

            String propValue = getPropString(fullKey);
            if (!propValue.isEmpty()) {
                try {
                    T value = parser.apply(propValue);
                    cache.put(fullKey, value);
                    return value;
                } catch (NumberFormatException e) {
                    System.out.println("WARNING: Error parsing numeric property " + fullKey + ": " + e.getMessage());
                } catch (IllegalArgumentException e) {
                    System.out.println("WARNING: Error parsing property " + fullKey + ": " + e.getMessage());
                }
            }
        } catch (SecurityException e) {
            System.out.println("WARNING: Security error accessing property " + fullKey + ": " + e.getMessage());
        }

        System.out.println("WARNING: default used for key " + fullKey);
        return defaultVal;
    }

    public static String getString(String prefix, String key, String defaultVal) {
        String result = getValueWithCache(key, prefix, defaultVal, STRING_MAP, str -> str);
        if (debug) {
            System.out.println("debug property " + prefix + "." + key + " = " + result);
        }
        return result;
    }

    public static double getNumber(String prefix, String key, Double defaultVal) {
        double result = getValueWithCache(key, prefix, defaultVal, NUMBER_MAP, Double::parseDouble);
        if (debug) {
            System.out.println("debug property " + prefix + "." + key + " = " + result);
        }
        return result;
    }

    public static int getInt(String prefix, String key, int defaultVal) {
        int result = getValueWithCache(key, prefix, defaultVal, INTEGER_MAP, Integer::parseInt);
        if (debug) {
            System.out.println("debug property " + prefix + "." + key + " = " + result);
        }
        return result;
    }

    public static boolean getBoolean(String prefix, String key, Boolean defaultVal) {
        boolean result = getValueWithCache(key, prefix, defaultVal, BOOLEAN_MAP,
                str -> str.equalsIgnoreCase("true"));
        if (debug) {
            System.out.println("debug property " + prefix + "." + key + " = " + result);
        }
        return result;
    }

    // Convenience methods for general prefix
    public static String getString(String key, String defaultVal) {
        return getString("general", key, defaultVal);
    }

    public static double getNumber(String key, Double defaultVal) {
        return getNumber("general", key, defaultVal);
    }

    public static int getInt(String key, int defaultVal) {
        return getInt("general", key, defaultVal);
    }

    public static boolean getBoolean(String key, Boolean defaultVal) {
        return getBoolean("general", key, defaultVal);
    }
}
