package frc.chargers.misc;

import edu.wpi.first.util.function.BooleanConsumer;
import lombok.Setter;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.function.DoubleConsumer;

/**
 * An API to handle tunable dashboard values.
 */
public class TunableValues {
	@Setter private static boolean tuningMode = false;

	public static class TunableNum extends LoggedNetworkNumber {
		private double defaultValue;
		private DoubleConsumer onChange = null;

		public TunableNum(String key, double defaultValue) {
			super("/Tuning/" + key, defaultValue);
			this.defaultValue = defaultValue;
		}

		public void onChange(DoubleConsumer impl) {
			onChange = impl;
		}

		@Override
		public void setDefault(double value) {
			super.setDefault(value);
			this.defaultValue = value;
		}

		@Override
		public double get() {
			return tuningMode ? super.get() : defaultValue;
		}

		@Override
		public void periodic() {
			super.periodic();
			if (tuningMode && onChange != null) onChange.accept(get());
		}
	}

	public static class TunableBool extends LoggedNetworkBoolean {
		private boolean defaultValue;
		private BooleanConsumer onChange = null;

		public TunableBool(String key, boolean defaultValue) {
			super("/Tuning/" + key, defaultValue);
			this.defaultValue = defaultValue;
		}

		public void onChange(BooleanConsumer impl) {
			onChange = impl;
		}

		@Override
		public void setDefault(boolean value) {
			super.setDefault(value);
			this.defaultValue = value;
		}

		@Override
		public boolean get() {
			return tuningMode ? super.get() : defaultValue;
		}

		@Override
		public void periodic() {
			super.periodic();
			if (tuningMode && onChange != null) onChange.accept(get());
		}
	}
}
