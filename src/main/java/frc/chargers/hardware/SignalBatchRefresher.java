package frc.chargers.hardware;

import com.ctre.phoenix6.BaseStatusSignal;

import java.util.HashSet;
import java.util.Set;

/**
 * A class that batch refreshes CTRE status signals to improve perf.
 */
public class SignalBatchRefresher {
    private static final Set<BaseStatusSignal> rioSignals = new HashSet<>();
    private static final Set<BaseStatusSignal> canivoreSignals = new HashSet<>();

    /**
     * Adds signals to the signal refresher.
     * This system exists because calling refreshAll() for multiple signals is faster
     * than calling refresh() for each signal.
     * @param isCanivore Whether a canivore is used. Most of the time, this is false.
     * @param signals The signals you want to add.
     */
    public static void register(boolean isCanivore, BaseStatusSignal... signals) {
        if (isCanivore) {
            canivoreSignals.addAll(Set.of(signals));
        } else {
            rioSignals.addAll(Set.of(signals));
        }
    }

    public static void unregister(BaseStatusSignal... signals) {
        canivoreSignals.removeAll(Set.of(signals));
        rioSignals.removeAll(Set.of(signals));
    }

    /**
     * Refreshes all signals. This must be called in robotPeriodic().
     */
    public static void refreshAll() {
        if (!rioSignals.isEmpty()) {
            BaseStatusSignal.refreshAll(rioSignals.toArray(new BaseStatusSignal[0]));
        }
        if (!canivoreSignals.isEmpty()) {
            BaseStatusSignal.refreshAll(canivoreSignals.toArray(new BaseStatusSignal[0]));
        }
    }
}
