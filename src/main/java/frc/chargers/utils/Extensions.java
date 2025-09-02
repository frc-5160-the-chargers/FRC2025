package frc.chargers.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.Collection;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * <h2>A collection of various generic utility methods.</h2>
 * <p>
 * If you add <code>@ExtensionMethod(Extensions.class)</code> on your class,
 * these functions can be used as extension methods, with the first parameter
 * as the receiver.
 * </p>
 * <p>
 * For instance, instead of <code>Extensions.methodName(a, b)</code>,
 * you can simply call <code>a.methodName(b)</code>.
 * </p>
 */
@SuppressWarnings("unused")
public class Extensions {
	private Extensions() {}

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
	
	public static int[] toIntArray(Collection<Integer> list) {
		return list.stream().mapToInt(Integer::intValue).toArray();
	}
	
	public static double[] toDoubleArray(Collection<Double> list) {
		return list.stream().mapToDouble(Double::doubleValue).toArray();
	}
}
