package frc.chargers.utils;

import com.ctre.phoenix6.BaseStatusSignal;
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
	private static final Set<BaseStatusSignal> allSignals = new HashSet<>();
	
	static {
		// creates a dummy subsystem to update signals
		new SubsystemBase() {
			@Override
			public void periodic() {
				var latestStatus = BaseStatusSignal.refreshAll(
					allSignals.stream()
						.filter(it -> it.getAppliedUpdateFrequency() == 50.0)
						.toArray(BaseStatusSignal[]::new)
				);
				GlobalLog.log("statusSignalLatestStatus", latestStatus.toString());
			}
		};
	}
	
	/** Refreshes the specified status signals automatically, at a rate of 0.02 seconds. */
	public static void register(BaseStatusSignal... signals) {
		allSignals.addAll(List.of(signals));
	}
	
	/** Prevents these signals from automatically refreshing. */
	public static void remove(BaseStatusSignal... signals) {
		for (var signal: signals) { allSignals.remove(signal); }
	}
}
