package frc.chargers.utils;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Setter;

import java.util.concurrent.atomic.AtomicReference;

/**
 * An API to read data from NetworkTables.
 * Can be used for replay, receiving data, or tunable values.
 */
public class TunableValues {
	@Setter private static boolean tuningMode = false;
	
	/** A number that can be tuned from the dashboard. */
	public static class TunableNum {
		private final DoubleEntry entry;
		private final double defaultValue;
		boolean respectTuningMode = false;
		
		public TunableNum(String path, double defaultValue) {
			this.entry = NetworkTableInstance
				             .getDefault()
				             .getDoubleTopic("/TunableValues/" + path)
				             .getEntry(defaultValue);
			entry.setDefault(defaultValue);
			this.defaultValue = defaultValue;
		}
		
		public double get() {
			return respectTuningMode && tuningMode ? entry.get(defaultValue) : defaultValue;
		}
		
		public Trigger changed() {
			if (!respectTuningMode) return new Trigger(() -> false);
			var previous = new AtomicReference<>(get());
			return new Trigger(() -> tuningMode && get() != previous.get());
		}
	}
	
	/** A boolean that can be tuned from the dashboard. */
	public static class TunableBool {
		private final BooleanEntry entry;
		private final boolean defaultValue;
		
		public TunableBool(String path, boolean defaultValue) {
			this.entry = NetworkTableInstance
				             .getDefault()
				             .getBooleanTopic("/TunableValues/" + path)
				             .getEntry(defaultValue);
			entry.setDefault(defaultValue);
			this.defaultValue = defaultValue;
		}
		
		public boolean get() {
			return tuningMode ? entry.get(defaultValue) : defaultValue;
		}
		
		public Trigger changed() {
			var previous = new AtomicReference<>(get());
			return new Trigger(() -> tuningMode && get() != previous.get());
		}
	}
}
