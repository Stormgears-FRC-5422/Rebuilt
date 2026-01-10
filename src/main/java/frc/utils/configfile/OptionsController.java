package frc.utils.configfile;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * Base class for managing a collection of options that can be modified through
 * Elastic dashboard. Each option is published to NetworkTables and can be
 * configured through Elastic widgets.
 */
public class OptionsController extends DashboardTuner {

    private final List<Option<?>> options = new ArrayList<>();
    private boolean initialized = false;
    private final String tableName;

    public OptionsController(String tableName) {
        super(tableName);
        this.tableName = tableName;
    }

    protected Option<Boolean> createBooleanOption(String label, boolean defaultValue) {
        Option<Boolean> option = new Option<>(label, defaultValue, tableName);
        options.add(option);
        return option;
    }

    protected Option<Double> createNumberOption(String label, double defaultValue) {
        Option<Double> option = new Option<>(label, defaultValue, tableName);
        options.add(option);
        return option;
    }

    protected Option<Integer> createIntegerOption(String label, int defaultValue) {
        Option<Integer> option = new Option<>(label, defaultValue, tableName);
        options.add(option);
        return option;
    }

    protected Option<String> createStringOption(String label, String defaultValue) {
        Option<String> option = new Option<>(label, defaultValue, tableName);
        options.add(option);
        return option;
    }

    @SuppressWarnings({"unchecked"})
    private void createTunables() {
        if (initialized) {
            return;
        }

        for (Option<?> option : options) {
            Object value = Objects.requireNonNull(option.get(), "Option value cannot be null");

            if (value instanceof Boolean boolVal) {
                var topic = table.getBooleanTopic(option.getLabel());
                var pub = topic.publish();
                var sub = topic.subscribe(boolVal);
                option.setPublisherAndSubscriber(pub, sub);
            } else if (value instanceof Double doubleVal) {
                var topic = table.getDoubleTopic(option.getLabel());
                var pub = topic.publish();
                var sub = topic.subscribe(doubleVal);
                option.setPublisherAndSubscriber(pub, sub);
            } else if (value instanceof Integer intVal) {
                var topic = table.getDoubleTopic(option.getLabel());
                var pub = topic.publish();
                var sub = topic.subscribe(intVal.doubleValue());
                option.setPublisherAndSubscriber(pub, sub);
            } else if (value instanceof String strVal) {
                var topic = table.getStringTopic(option.getLabel());
                var pub = topic.publish();
                var sub = topic.subscribe(strVal);
                option.setPublisherAndSubscriber(pub, sub);
            } else {
                throw new IllegalArgumentException(
                        "Unsupported option type: " + (value != null ? value.getClass().getName() : "null"));
            }
        }
        initialized = true;
    }

    @SuppressWarnings("unchecked")
    public void periodic() {
        if (!initialized) {
            createTunables();
            return;
        }

        for (Option<?> option : options) {
            Object value = Objects.requireNonNull(option.get(), "Option value cannot be null");

            if (value instanceof Boolean) {
                ((Option<Boolean>) option).updateValue(option.getBooleanSubscriber().get());
            } else if (value instanceof Double) {
                ((Option<Double>) option).updateValue(option.getDoubleSubscriber().get());
            } else if (value instanceof Integer) {
                ((Option<Integer>) option).updateValue((int) option.getDoubleSubscriber().get());
            } else if (value instanceof String) {
                ((Option<String>) option).updateValue(option.getStringSubscriber().get());
            } else {
                throw new IllegalArgumentException(
                        "Unsupported option type: " + (value != null ? value.getClass().getName() : "null"));
            }
        }
    }
}
