package frc.chargers.utils;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Setter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * An API to handle tunable dashboard values.
 */
public class TunableValues {
	@Setter private static boolean tuningMode = false;
	
	/** A number that can be tuned from the dashboard. */
	public static class TunableNum implements DoubleSupplier {
		private final DoubleEntry entry;
		private double defaultValue;
		private double previousValue;
		
		public TunableNum(String path, double defaultValue) {
			this.entry = NetworkTableInstance
				             .getDefault()
				             .getDoubleTopic("/TunableValues/" + path)
				             .getEntry(defaultValue);
			entry.setDefault(defaultValue);
			this.defaultValue = defaultValue;
			this.previousValue = defaultValue;
		}
		
		public void onChange(Runnable toRun) {
			new Trigger(() -> {
				if (!tuningMode) return false;
				var latest = get();
				boolean hasChanged = latest != previousValue;
				previousValue = latest;
				return hasChanged;
			}).onTrue(Commands.runOnce(toRun).ignoringDisable(true));
		}
		
		public void setDefault(double defaultValue) {
			this.defaultValue = defaultValue;
		}
		
		public double get() {
			return tuningMode ? entry.get(defaultValue) : defaultValue;
		}
		
		@Override
		public double getAsDouble() {
			return get();
		}
	}
	
	/** A boolean that can be tuned from the dashboard. */
	public static class TunableBool implements BooleanSupplier {
		private final BooleanEntry entry;
		private boolean defaultValue;
		private boolean previousValue = false;
		
		public TunableBool(String path, boolean defaultValue) {
			this.entry = NetworkTableInstance
				             .getDefault()
				             .getBooleanTopic("/TunableValues/" + path)
				             .getEntry(defaultValue);
			entry.setDefault(defaultValue);
			this.defaultValue = defaultValue;
		}
		
		public void onChange(Runnable toRun) {
			new Trigger(() -> {
				if (!tuningMode) return false;
				var latest = get();
				boolean hasChanged = latest != previousValue;
				previousValue = latest;
				return hasChanged;
			}).onTrue(Commands.runOnce(toRun).ignoringDisable(true));
		}
		
		public void setDefault(boolean defaultValue) {
			this.defaultValue = defaultValue;
		}
		
		public boolean get() {
			return tuningMode ? entry.get(defaultValue) : defaultValue;
		}
		
		@Override
		public boolean getAsBoolean() {
			return get();
		}
	}
}
