package frc.chargers.misc;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.util.function.BooleanConsumer;
import lombok.Setter;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

/**
 * An API to handle tunable dashboard values.
 */
public class TunableValues {
	@Setter private static boolean tuningMode = false;

	public static class TunableNum extends LoggedNetworkNumber {
		private double value;
		private DoubleConsumer onChange = null;

		public TunableNum(String key, double value) {
			super("/Tuning/" + key, value);
			this.value = value;
		}

		public void onChange(DoubleConsumer impl) {
			onChange = impl;
		}

		@Override
		public void setDefault(double value) {
			super.setDefault(value);
			this.value = value;
		}

		@Override
		public double get() {
			return tuningMode ? super.get() : value;
		}

		@Override
		public void periodic() {
			super.periodic();
			if (tuningMode && onChange != null) {
				var latest = get();
				if (latest == value) return;
				onChange.accept(latest);
				value = latest;
			}
		}
	}

	public static class TunableBool extends LoggedNetworkBoolean {
		private boolean value;
		private BooleanConsumer onChange = null;

		public TunableBool(String key, boolean value) {
			super("/Tuning/" + key, value);
			this.value = value;
		}

		public void onChange(BooleanConsumer impl) {
			onChange = impl;
		}

		@Override
		public void setDefault(boolean value) {
			super.setDefault(value);
			this.value = value;
		}

		@Override
		public boolean get() {
			return tuningMode ? super.get() : value;
		}

		@Override
		public void periodic() {
			super.periodic();
			if (tuningMode && onChange != null) {
				var latest = get();
				if (latest == value) return;
				onChange.accept(latest);
				value = latest;
			}
		}
	}

	/** Represents a Tunable measure(Distance, Angle, etc.) */
	public static class Tunable<M extends Measure<?>> {
		private M value;
		private Consumer<M> onChange = null;
		private final Unit unit;
		private final LoggedNetworkNumber inner;

		public Tunable(String key, M value) {
			inner = new LoggedNetworkNumber(
				"/Tuning/" + key + "(" + value.unit().name() + ")",
				value.magnitude()
			) {
				@Override
				public void periodic() {
					super.periodic();
					if (tuningMode && onChange != null) {
						onChange.accept(Tunable.this.get());
					}
				}
			};
			this.value = value;
			this.unit = value.unit();
		}

		public void onChange(Consumer<M> impl) {
			onChange = impl;
		}

		public void setDefault(M value) {
			inner.setDefault(value.magnitude());
			this.value = value;
		}

		@SuppressWarnings("unchecked")
		public M get() {
			return tuningMode ? (M) unit.of(inner.get()) : value;
		}
	}
}
