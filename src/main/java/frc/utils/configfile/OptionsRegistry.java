package frc.utils.configfile;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class OptionsRegistry {

    private static final Map<Class<?>, OptionsController> instances = new HashMap<>();
    private static final List<OptionsController> controllers = new ArrayList<>();

    public static void register(OptionsController controller) {
        controllers.add(controller);
        instances.put(controller.getClass(), controller);
    }

    public static void periodic() {
        for (OptionsController controller : controllers) {
            controller.periodic();
        }
    }

    @SuppressWarnings("unchecked")
    public static <T extends OptionsController> T getInstance(Class<T> type) {
        return (T) instances.get(type);
    }
}
