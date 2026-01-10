package frc.utils.configfile;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;

import java.util.Objects;

/**
 * Represents a single configurable option that can be modified through Elastic
 * dashboard. Each option has a value, label, and tracks whether it has been
 * modified.
 *
 * @param <T> The type of the option value (Boolean, Double, Integer, or String)
 */
public class Option<T> {

    private final String label;
    private final String tableName;
    private T value;
    private boolean dirty;
    private Object publisher;
    private Object subscriber;

    public Option(String label, T defaultValue, String tableName) {
        this.label = label;
        this.value = defaultValue;
        this.tableName = tableName;
        this.dirty = false;
    }

    /**
     * Get the current value of the option
     */
    public T get() {
        return value;
    }

    /**
     * Implicit conversion to the underlying type
     */
    public T value() {
        return value;
    }

    /**
     * Update the value and mark as dirty if changed
     */
    void updateValue(T newValue) {
        boolean changed;
        if (value instanceof Boolean && newValue instanceof Boolean) {
            // Direct comparison for booleans to avoid any boxing issues
            changed = ((Boolean) value).booleanValue() != ((Boolean) newValue).booleanValue();
        } else {
            changed = !Objects.equals(newValue, value);
        }

        if (changed) {
            value = newValue;
            dirty = true;
            updatePublisher();
            console("updated to " + value);
        }
    }

    private void updatePublisher() {
        if (publisher == null || value == null) {
            return;
        }

        if (value instanceof Boolean boolVal) {
            ((BooleanPublisher) publisher).set(boolVal);
        } else if (value instanceof Double doubleVal) {
            ((DoublePublisher) publisher).set(doubleVal);
        } else if (value instanceof Integer intVal) {
            ((DoublePublisher) publisher).set((double) intVal);
        } else if (value instanceof String strVal) {
            ((StringPublisher) publisher).set(strVal);
        } else {
            throw new IllegalArgumentException(
                    "Unsupported option type: " + (value != null ? value.getClass().getName() : "null"));
        }
    }

    /**
     * Check if the value has been modified since last clearDirty()
     */
    public boolean isDirty() {
        return dirty;
    }

    /**
     * Clear the dirty flag. This should be called by the developer after
     * handling the change.
     */
    public void clearDirty() {
        dirty = false;
    }

    String getLabel() {
        return label;
    }

    void setPublisherAndSubscriber(Object publisher, Object subscriber) {
        this.publisher = publisher;
        this.subscriber = subscriber;
        updatePublisher(); // Publish initial value
    }

    BooleanSubscriber getBooleanSubscriber() {
        return (BooleanSubscriber) subscriber;
    }

    DoubleSubscriber getDoubleSubscriber() {
        return (DoubleSubscriber) subscriber;
    }

    StringSubscriber getStringSubscriber() {
        return (StringSubscriber) subscriber;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == null) {
            return false;
        }
        if (obj == this) {
            return true;
        }
        if (obj instanceof Option<?> other) {
            return value.equals(other.value);
        }
        return value.equals(obj);
    }

    @Override
    public int hashCode() {
        return value.hashCode();
    }

    @Override
    public String toString() {
        return value.toString();
    }

    private void console(String message) {
        System.out.println("Option [" + tableName + "/" + label + "]: " + message);
    }

    /**
     * Set a new value and publish it to NetworkTables
     *
     * @param newValue The new value to set
     */
    public void setValue(T newValue) {
        updateValue(newValue);
    }

}
