package frc.chargers.utils;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import static monologue.Monologue.GlobalLog;

/**
 * Serves to update status signals all at once instead of using separate refresh() calls.
 * This reduces latency.
 */
public class SignalAutoRefresher {
	private static final Set<StatusSignal<?>> allSignals = new HashSet<>();
	private static final StatusSignal<?>[] dummyArray = {};
	
	static {
		// creates a dummy subsystem to update signals
		new SubsystemBase() {
			@Override
			public void periodic() {
				var latestStatus = BaseStatusSignal.refreshAll(allSignals.toArray(dummyArray));
				GlobalLog.log("statusSignalLatestStatus", latestStatus.toString());
			}
		};
	}
	
	/** Refreshes the specified status signals automatically, at a rate of 0.02 seconds. */
	public static void register(StatusSignal<?>... signals) {
		allSignals.addAll(List.of(signals));
	}
	
	/** Prevents these signals from automatically refreshing. */
	public static void remove(StatusSignal<?>... signals) {
		for (var signal: signals) { allSignals.remove(signal); }
	}
}
