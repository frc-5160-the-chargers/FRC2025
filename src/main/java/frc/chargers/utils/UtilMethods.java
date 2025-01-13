package frc.chargers.utils;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.RequiredArgsConstructor;

import java.util.Collection;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

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
	
	/** "Binds" an alert to a boolean supplier; pushing it to the dashboard when the condition returns true. */
	public static void bind(Alert alert, BooleanSupplier condition) {
		var trigger = condition instanceof Trigger t ? t : new Trigger(condition);
		trigger
			.onTrue(Commands.runOnce(() -> alert.set(true)))
			.onFalse(Commands.runOnce(() -> alert.set(false)));
	}
	
	/**
	 * Gets a trigger that returns true once when the value changes.
	 */
	public static Trigger hasChanged(Supplier<?> supplier) {
		var hasChangedHandler = new HasChangedHandler(supplier);
		return new Trigger(hasChangedHandler::compute);
	}
	
	/**
	 * Creates a new trigger that returns true when the receiver is double-clicked.
	 * <p>
	 * Usage: <code>doubleClicked(controller.x()).onTrue(command)</code> <br />
	 * With @ExtensionMethod: <code>controller.x().doubleClicked().onTrue(command)</code>
	 * </p>
	 */
	public static Trigger doubleClicked(Trigger receiver) {
		return doubleClicked(receiver, 0.4);
	}
	
	/**
	 * Creates a new trigger that returns true when the receiver is double-clicked.
	 */
	public static Trigger doubleClicked(Trigger receiver, double maxLengthSeconds) {
		var tracker = new DoublePressTracker(receiver, maxLengthSeconds);
		return new Trigger(tracker::get);
	}
	
	/** Runs the toRun method immediately after the time has elapsed. */
	public static void waitThenRun(Time time, Runnable toRun) {
		Commands.waitTime(time)
			.andThen(Commands.runOnce(toRun))
			.ignoringDisable(true)
			.schedule();
	}
	
	/** Configures a spark motor with default ResetMode and PersistMode options. */
	public static REVLibError resetAndConfigure(SparkBase receiver, SparkBaseConfig config) {
		return receiver.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
	}
	
	/** Gets the distance between 2 Pose2d's. */
	public static double getDistance(Pose2d start, Pose2d end) {
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
	
	@RequiredArgsConstructor
	private static class HasChangedHandler {
		private final Supplier<?> supplier;
		private Object value = null;
		
		public boolean compute() {
			var current = supplier.get();
			if (value != current) {
				value = current;
				return value != null;
			} else {
				return false;
			}
		}
	}
	
	@RequiredArgsConstructor
	private static class DoublePressTracker {
		private final Trigger trigger;
		private final double maxLengthSecs;
		
		private final Timer resetTimer = new Timer();
		private DoublePressState state = DoublePressState.IDLE;
		
		public boolean get() {
			boolean pressed = trigger.getAsBoolean();
			switch (state) {
				case IDLE:
					if (pressed) {
						state = DoublePressState.FIRST_PRESS;
						resetTimer.reset();
						resetTimer.start();
					}
					break;
				case FIRST_PRESS:
					if (!pressed) {
						if (resetTimer.hasElapsed(maxLengthSecs)) {
							reset();
						} else {
							state = DoublePressState.FIRST_RELEASE;
						}
					}
					break;
				case FIRST_RELEASE:
					if (pressed) {
						state = DoublePressState.SECOND_PRESS;
					} else if (resetTimer.hasElapsed(maxLengthSecs)) {
						reset();
					}
					break;
				case SECOND_PRESS:
					if (!pressed) {
						reset();
					}
			}
			return state == DoublePressState.SECOND_PRESS;
		}
		
		private void reset() {
			state = DoublePressState.IDLE;
			resetTimer.stop();
		}
	}
	
	private enum DoublePressState {
		IDLE,
		FIRST_PRESS,
		FIRST_RELEASE,
		SECOND_PRESS
	}
}
