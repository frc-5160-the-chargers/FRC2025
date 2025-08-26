package frc.chargers.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

/**
 * Calling {@link BaseStatusSignal#refreshAll(BaseStatusSignal...)} has a measurable increase in
 * performance due to reduced JNI and data read/write overhead compared to calling {@link
 * StatusSignal#refresh()} on individual signals.
 *
 * <p>To maximize performance, all {@link BaseStatusSignal}s that aren't on separate threads are
 * batched together into one call via a global state in this class.
 *
 * <p>Kinda ugly since it becomes way less clear when data is being updated and global state stuff
 * is hard to debug but sometimes you gotta prioritize the RoboRIO's happiness over your own.
 */
public final class PhoenixBatchRefresher {
    private PhoenixBatchRefresher() {}

    private static BaseStatusSignal[] registeredCanivoreSignals = new BaseStatusSignal[0];
    private static BaseStatusSignal[] registeredRioSignals = new BaseStatusSignal[0];

    public static void refresh() {
        BaseStatusSignal.refreshAll(registeredCanivoreSignals);
        BaseStatusSignal.refreshAll(registeredRioSignals);
    }

    public static void registerCanivore(BaseStatusSignal... signals) {
        var newSignals = new BaseStatusSignal[registeredCanivoreSignals.length + signals.length];
        System.arraycopy(registeredCanivoreSignals, 0, newSignals, 0, registeredCanivoreSignals.length);
        System.arraycopy(signals, 0, newSignals, registeredCanivoreSignals.length, signals.length);
        registeredCanivoreSignals = newSignals;
    }

    public static void registerRio(BaseStatusSignal... signals) {
        var newSignals = new BaseStatusSignal[registeredRioSignals.length + signals.length];
        System.arraycopy(registeredRioSignals, 0, newSignals, 0, registeredRioSignals.length);
        System.arraycopy(signals, 0, newSignals, registeredRioSignals.length, signals.length);
        registeredRioSignals = newSignals;
    }
}
