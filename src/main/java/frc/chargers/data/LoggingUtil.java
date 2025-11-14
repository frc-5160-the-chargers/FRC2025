package frc.chargers.data;

import org.littletonrobotics.junction.Logger;

import java.util.Collection;

/** Provides utilities for logging java lists as outputs. */
public class LoggingUtil {
    public static void logDoubleList(String key, Collection<Double> values) {
        var array = new double[values.size()];
        int i = 0;
        for (var value: values) array[i++] = value;
        Logger.recordOutput(key, array);
    }

    public static void logIntList(String key, Collection<Integer> values) {
        var array = new int[values.size()];
        int i = 0;
        for (var value: values) array[i++] = value;
        Logger.recordOutput(key, array);
    }
}
