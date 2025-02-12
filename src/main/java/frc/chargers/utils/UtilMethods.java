package frc.chargers.utils;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.Collection;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import static edu.wpi.first.wpilibj.Alert.AlertType.kError;

/**
 * <h2>A collection of various utility methods.</h2>
 * <p>
 * With <code>@ExtensionMethod(UtilMethods.class)</code> on your class,
 * most of these functions(excluding equivalent() and createList())
 * can be converted to extension methods as well.
 * </p>
 * <p>
 * For instance, instead of <code>UtilMethods.methodName(a, b)</code>,
 * you can simply call <code>a.methodName(b)</code>.
 * </p>
 */
@SuppressWarnings("unused")
public class UtilMethods {
	private UtilMethods() {}
	private static final double EPSILON = 1E-9;
	private static final int MAX_CONFIG_RETRIES = 10;
	
	/**
	 * Checks if 2 doubles are equal; correcting for floating point error.
	 * Note: cannot be used as extension due to the double type being primitive.
	 * Usage: <code>equivalent(2.0, 3.0 - 1.0)</code>
	 */
	public static boolean equivalent(double a, double b) {
		return Math.abs(a - b) <= EPSILON;
	}
	
	/**
	 * Runs toRun with the receiver, then returns the receiver.
	 * Intended to be used as an extension method via @ExtensionMethod(UtilMethods.class)
	 * Usage: <code>someValue.also(it -> it.instanceMethod());</code>
	 */
	public static <T> T also(T receiver, Consumer<T> toRun) {
		toRun.accept(receiver);
		return receiver;
	}
	
	/** Runs the toRun method immediately after the time has elapsed. */
	public static void waitThenRun(double delaySecs, Runnable toRun) {
		Commands.waitSeconds(delaySecs)
			.andThen(Commands.runOnce(toRun))
			.ignoringDisable(true)
			.schedule();
	}
	
	/** Gets the distance between 2 Pose2d's. */
	public static double distanceBetween(Pose2d start, Pose2d end) {
		return start.getTranslation().getDistance(end.getTranslation());
	}
	
	@SuppressWarnings("unchecked")
	public static <T> List<T> createList(int size, Supplier<T> generator) {
		var arr = new Object[size];
		for (int i = 0; i < size; i++) arr[i] = generator.get();
		return List.of((T[]) arr);
	}
	
	public static int[] toIntArray(Collection<Integer> list) {
		var arr = new int[list.size()];
		int idx = 0;
		for (var value: list) {
			arr[idx] = value;
			idx++;
		}
		return arr;
	}
	
	public static double[] toDoubleArray(Collection<Double> list) {
		var arr = new double[list.size()];
		int idx = 0;
		for (var value: list) {
			arr[idx] = value;
			idx++;
		}
		return arr;
	}
	
	/** Runs a CTRE configure call up to 4 times, stopping if it returns an ok status. */
	public static StatusCode tryUntilOk(ParentDevice device, Supplier<StatusCode> configureFn) {
		var result = StatusCode.OK;
		for (int i = 0; i < MAX_CONFIG_RETRIES; i++) {
			result = configureFn.get();
			if (result.isOK()) return result;
		}
		new Alert(device.getClass().getSimpleName() + " with id " + device.getDeviceID() + " didn't configure: " + result, kError)
			.set(true);
		return result;
	}
	
	public static REVLibError tryUntilOk(SparkBase device, Supplier<REVLibError> configureFn) {
		var result = REVLibError.kOk;
		for (int i = 0; i < MAX_CONFIG_RETRIES; i++) {
			result = configureFn.get();
			if (result == REVLibError.kOk) return result;
		}
		new Alert("Spark with id " + device.getDeviceId() + " didn't configure: " + result, kError).set(true);
		return result;
	}
}
