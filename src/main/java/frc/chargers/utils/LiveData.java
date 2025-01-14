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
public class LiveData {
	@Setter private static boolean tuningMode = false;
	
	/** A wrapper over networktables that reads live double values. */
	public static class NTDouble {
		private final DoubleEntry entry;
		private final double defaultValue;
		boolean respectTuningMode = false;
		
		/**
		 * Creates a NTDouble that always returns its default value
		 * unless LiveData.setTuningMode(true) is called.
		 * It also places the path under the "Tunables" section.
		 */
		public static NTDouble asTunable(String path, double defaultValue) {
			var obj = new NTDouble("TunableValues/" + path, defaultValue);
			obj.respectTuningMode = true;
			return obj;
		}
		
		public NTDouble(String path, double defaultValue) {
			this.entry = NetworkTableInstance
				             .getDefault()
				             .getDoubleTopic(path)
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
			return new Trigger(() -> get() != previous.get());
		}
	}
	
	public static class NTBoolean {
		private final BooleanEntry entry;
		private final boolean defaultValue;
		boolean respectTuningMode = false;
		
		/**
		 * Creates a NTBoolean that always returns its default value
		 * unless LiveData.setTuningMode(true) is called.
		 */
		public static NTBoolean asTunable(String path, boolean defaultValue) {
			var obj = new NTBoolean("TunableValues/" + path, defaultValue);
			obj.respectTuningMode = true;
			return obj;
		}
		
		public NTBoolean(String path, boolean defaultValue) {
			this.entry = NetworkTableInstance
				             .getDefault()
				             .getBooleanTopic(path)
				             .getEntry(defaultValue);
			entry.setDefault(defaultValue);
			this.defaultValue = defaultValue;
		}
		
		public boolean get() {
			return respectTuningMode && tuningMode ? entry.get(defaultValue) : defaultValue;
		}
		
		public Trigger changed() {
			if (!respectTuningMode) return new Trigger(() -> false);
			var previous = new AtomicReference<>(get());
			return new Trigger(() -> get() != previous.get());
		}
	}
}
