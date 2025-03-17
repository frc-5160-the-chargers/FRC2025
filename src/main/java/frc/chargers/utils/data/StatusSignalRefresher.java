package frc.chargers.utils.data;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.chargers.utils.Tracer;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import static monologue.Monologue.GlobalLog;

/**
 * A utility class that allows for all the robot's CTRE StatusSignals
 * to be refreshed at once. This reduces JNI overhead.
 * You must have ```StatusSignalRefresher.startPeriodic(this)``` in all Robot classes.
 */
public class StatusSignalRefresher {
	private StatusSignalRefresher() {}
	
	private static BaseStatusSignal[] statusSignalsAsArray = null;
	private static final Set<BaseStatusSignal> statusSignals = new HashSet<>();
	
	/**
	 * An alternative to calling StatusSignalRefresher.periodic() within the robotPeriodic
	 * method of TimedRobot.
	 */
	public static void startPeriodic(TimedRobot robot) {
		robot.addPeriodic(StatusSignalRefresher::periodic, 0.02);
	}
	
	/**
	 * You must run this method periodically for ChargerTalonFX's and ChargerCANcoder's to work.
	 * Alternatively, use startPeriodic(this) in your Robot class instead.
	 */
	public static void periodic() {
		Tracer.startTrace("CAN signal refresh");
		if (statusSignalsAsArray == null || statusSignalsAsArray.length != statusSignals.size()) {
			statusSignalsAsArray = statusSignals.toArray(new BaseStatusSignal[0]);
		}
		var latestStatus = BaseStatusSignal.refreshAll(statusSignalsAsArray);
		GlobalLog.log("refreshStatus", latestStatus.toString());
		Tracer.endTrace();
	}
	
	/** Refreshes the specified status signals automatically, at a rate of 0.02 seconds. */
	public static void addSignals(BaseStatusSignal... signals) {
		for (var signal: signals) {
			if (signal != null) statusSignals.add(signal);
		}
		statusSignals.addAll(List.of(signals));
	}
	
	public static void remove(BaseStatusSignal... signals) {
		List.of(signals).forEach(statusSignals::remove);
	}
}
