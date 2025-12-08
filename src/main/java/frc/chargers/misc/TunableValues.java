package frc.chargers.misc;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.util.function.BooleanConsumer;
import lombok.Setter;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Radians;

/**
 * An API to handle tunable dashboard values.
 * {@link TunableNum}, {@link TunableBool}, and {@link Tunable<>} are volatile,
 * meaning that the return value of get() is constantly changing; thus, you should use
 * the onChange() instance methods accordingly.
 */
public class TunableValues {
	@Setter private static boolean tuningMode = false;

	/** Represents a tunable double. */
	public static class TunableNum extends LoggedNetworkNumber {
		private double value;
		private final List<DoubleConsumer> listeners = new ArrayList<>();

		public TunableNum(String key, double value) {
			super("/Tuning/" + key, value);
			this.value = value;
		}

		public void onChange(DoubleConsumer impl) {
			listeners.add(impl);
		}

		@Override
		public void setDefault(double value) {
			super.setDefault(value);
			this.value = value;
		}

		@Override
		public double get() {
			return value;
		}

		@Override
		public void periodic() {
			super.periodic();
			if (!tuningMode) return;
			double valueFromNt = super.get();
			if (valueFromNt != value) {
				value = valueFromNt;
				for (var listener : listeners) {
					listener.accept(value);
				}
			}
		}
	}

	/** Represents a tunable boolean. */
	public static class TunableBool extends LoggedNetworkBoolean {
		private boolean value;
		private final List<BooleanConsumer> listeners = new ArrayList<>();

		public TunableBool(String key, boolean value) {
			super("/Tuning/" + key, value);
			this.value = value;
		}

		public void onChange(BooleanConsumer impl) {
			listeners.add(impl);
		}

		@Override
		public void setDefault(boolean value) {
			super.setDefault(value);
			this.value = value;
		}

		@Override
		public boolean get() {
			return value;
		}

		@Override
		public void periodic() {
			super.periodic();
			if (!tuningMode) return;
			boolean valueFromNt = super.get();
			if (valueFromNt != value) {
				value = valueFromNt;
				for (var listener: listeners) {
					listener.accept(valueFromNt);
				}
			}
		}
	}

	/** Represents a Tunable measure(Distance, Angle, etc.) */
	public static class Tunable<M extends Measure<?>> {
		private M value;
		private final List<Consumer<M>> listeners = new ArrayList<>();
		private final LoggedNetworkNumber inner;

		public Tunable(String key, M value) {
			this.value = value;
			inner = new LoggedNetworkNumber(
				"/Tuning/" + key + "(" + value.unit().name() + ")",
				value.magnitude()
			) {
				@Override
				public void periodic() {
					super.periodic();
					handleOnChange();
				}
			};
		}

		@SuppressWarnings("unchecked")
		private void handleOnChange() {
			if (!tuningMode) return;
			double valueFromNt = inner.get();
			if (value.magnitude() != valueFromNt) {
				value = (M) value.unit().of(valueFromNt);
				for (var listener: listeners) {
					listener.accept(value);
				}
			}
		}

		public void onChange(Consumer<M> impl) {
			listeners.add(impl);
		}

		public M get() {
			return value;
		}
	}
}
