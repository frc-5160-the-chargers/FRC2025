package frc.chargers.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import frc.chargers.misc.Tracer;

import java.util.ArrayList;
import java.util.List;

/**
 * A class that batch refreshes CTRE status signals to improve perf.
 */
public class SignalBatchRefresher {
    private static final List<BaseStatusSignal> rioSignals = new ArrayList<>();
    private static final List<BaseStatusSignal> canivoreSignals = new ArrayList<>();

    /**
     * Adds signals to the signal refresher.
     * This system exists because calling refreshAll() for multiple signals is faster
     * than calling refresh() for each signal.
     * @param isCanivore Whether a canivore is used. Most of the time, this is false.
     * @param signals The signals you want to add.
     */
    public static void register(boolean isCanivore, BaseStatusSignal... signals) {
        if (isCanivore) {
            canivoreSignals.addAll(List.of(signals));
        } else {
            rioSignals.addAll(List.of(signals));
        }
    }

    public static void unregister(BaseStatusSignal... signals) {
        canivoreSignals.removeAll(List.of(signals));
        rioSignals.removeAll(List.of(signals));
    }

    /**
     * Refreshes all signals. This must be called in robotPeriodic().
     */
    public static void refreshAll() {
        Tracer.startTrace("Signal Refresh");
        if (!rioSignals.isEmpty()) BaseStatusSignal.refreshAll(rioSignals);
        if (!canivoreSignals.isEmpty()) BaseStatusSignal.refreshAll(canivoreSignals);
        Tracer.endTrace();
    }
}
