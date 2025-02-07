package frc.chargers.utils;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.wpilibj.TimedRobot;
import lombok.Getter;

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
	
	@Getter private static final Set<BaseStatusSignal> statusSignals = new HashSet<>();
	private static final BaseStatusSignal[] dummyArray = new BaseStatusSignal[0];
	
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
		var latestStatus = BaseStatusSignal.refreshAll(statusSignals.toArray(dummyArray));
		GlobalLog.log("refreshStatus", latestStatus.toString());
	}
	
	/** Refreshes the specified status signals automatically, at a rate of 0.02 seconds. */
	public static void addSignals(BaseStatusSignal... signals) {
		statusSignals.addAll(List.of(signals));
	}
}
