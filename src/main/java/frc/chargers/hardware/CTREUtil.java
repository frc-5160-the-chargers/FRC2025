package frc.chargers.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.traits.CommonTalon;
import com.ctre.phoenix6.hardware.traits.CommonTalonWithFOC;
import edu.wpi.first.wpilibj.Alert;
import org.littletonrobotics.junction.Logger;

import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import static edu.wpi.first.wpilibj.Alert.AlertType.kError;

public class CTREUtil {
	private CTREUtil() {}

	/**
	 * Attempts to run the command until no error is produced.
	 * If the max attempts are exceeded, an alert will be shown
	 * unless if errorMsg is null.
	 */
	public static StatusCode retryFor(
		int maxAttempts,
		String errorMsg,
		Supplier<StatusCode> command
	) {
		var result = StatusCode.OK;
		for (int i = 0; i < maxAttempts; i++) {
			result = command.get();
			if (result.isOK()) return result;
		}
		if (errorMsg != null) {
			new Alert("[" + result + "] " + errorMsg, kError).set(true);
		}
		return result;
	}

	private static BaseStatusSignal[] rioSignalsAsArray = {};
	private static final Set<BaseStatusSignal> rioSignals = new HashSet<>();

	private static BaseStatusSignal[] canivoreSignalsAsArray = {};
	private static final Set<BaseStatusSignal> canivoreSignals = new HashSet<>();

	/**
	 * This method must be called in a TimedRobot's robotPeriodic() method.
	 * Otherwise, MotorData updating will not work.
	 */
	public static void refreshSignals() {
		if (rioSignalsAsArray.length > 0) {
			var rioStatus = BaseStatusSignal.refreshAll(rioSignalsAsArray);
			Logger.recordOutput("Batch Signal Refresh/rio", rioStatus);
		}
		if (canivoreSignalsAsArray.length > 0) {
			var canivoreStatus = BaseStatusSignal.refreshAll(canivoreSignalsAsArray);
			Logger.recordOutput("Batch Signal Refresh/canivore", canivoreStatus);
		}
	}
	
	/** Refreshes the specified status signals automatically, at a rate of 0.02 seconds. */
	public static void addSignals(boolean isCanivore, BaseStatusSignal... signals) {
		retryFor(
			4, "Signal frequencies not set",
			() -> BaseStatusSignal.setUpdateFrequencyForAll(50, signals)
		);
		if (isCanivore) {
			canivoreSignals.addAll(List.of(signals));
			canivoreSignalsAsArray = canivoreSignals.toArray(new BaseStatusSignal[0]);
		} else {
			rioSignals.addAll(List.of(signals));
			rioSignalsAsArray = rioSignals.toArray(new BaseStatusSignal[0]);
		}
	}

	// Internals for MotorData API
	private static final Set<CommonTalon> registeredMotors = new HashSet<>();

	static void addLeaderMotorSignals(boolean isCanivore, CommonTalon motor) {
		if (registeredMotors.add(motor)) {
			addSignals(
				isCanivore,
				motor.getPosition(),
				motor.getVelocity(),
				motor.getSupplyVoltage(),
				motor.getSupplyCurrent(),
				motor.getDeviceTemp(),
				motor.getTorqueCurrent()
			);
		}
	}

	static void addFollowerMotorSignals(boolean isCanivore, CommonTalon... followers) {
		for (var motor : followers) {
			if (!registeredMotors.add(motor)) continue;
			addSignals(
				isCanivore,
				motor.getSupplyVoltage(),
				motor.getSupplyCurrent(),
				motor.getDeviceTemp(),
				motor.getTorqueCurrent()
			);
		}
	}
}
