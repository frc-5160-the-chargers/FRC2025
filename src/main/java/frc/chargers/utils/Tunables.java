package frc.chargers.utils;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Setter;

import java.util.concurrent.atomic.AtomicReference;

@SuppressWarnings("unused")
public class Tunables {
	/** Whether to listen to networktables values. */
	@Setter private static boolean tuningMode = false;
	
	public static class TunableDouble {
		private final DoubleEntry entry;
		private final double defaultValue;
		
		public TunableDouble(String path, double defaultValue) {
			this.entry = NetworkTableInstance
				             .getDefault()
				             .getDoubleTopic("Tuning/" + path)
				             .getEntry(defaultValue);
			entry.setDefault(defaultValue);
			this.defaultValue = defaultValue;
		}
		
		private TunableDouble(DoubleEntry entry, double defaultValue) {
			this.entry = entry;
			this.defaultValue = defaultValue;
		}
		
		public double get() {
			return tuningMode ? entry.get(defaultValue) : defaultValue;
		}
		
		public Trigger changed() {
			var previous = new AtomicReference<>(get());
			return new Trigger(() -> get() != previous.get());
		}
	}
	
	public static class TunableBoolean {
		private final BooleanEntry entry;
		private final boolean defaultValue;
		
		public TunableBoolean(String path, boolean defaultValue) {
			this.entry = NetworkTableInstance
				             .getDefault()
				             .getBooleanTopic("Tuning/" + path)
				             .getEntry(defaultValue);
			entry.setDefault(defaultValue);
			this.defaultValue = defaultValue;
		}
		
		private TunableBoolean(BooleanEntry entry, boolean defaultValue) {
			this.entry = entry;
			this.defaultValue = defaultValue;
		}
		
		public boolean get() {
			return tuningMode ? entry.get(defaultValue) : defaultValue;
		}
		
		public Trigger changed() {
			var previous = new AtomicReference<>(get());
			return new Trigger(() -> get() != previous.get());
		}
	}
}
