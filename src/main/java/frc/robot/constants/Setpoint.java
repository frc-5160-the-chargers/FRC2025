package frc.robot.constants;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import java.util.Map;

import static edu.wpi.first.units.Units.*;
import static java.util.Map.entry;

/** Represents an elevator + wrist setpoint. */
public record Setpoint(Distance elevatorHeight, Angle wristTarget, String name) {
	private static final Map<Integer, Setpoint> LEVEL_TO_SETPOINT_MAP = Map.ofEntries(
		entry(1, new Setpoint(Meters.zero(), Degrees.zero(), "L1")),
		entry(2, new Setpoint(Meters.of(0.11), Degrees.of(0), "L2")),
		entry(3, new Setpoint(Meters.of(0.52), Degrees.of(0), "L3")),
		entry(4, new Setpoint(Meters.of(1.27), Degrees.of(12), "L4")) // highest possible elevator setpoint
	);
	
	public static Setpoint score(int level) {
		if (level < 1 || level > 4) {
			throw new RuntimeException("Invalid level: " + level);
		}
		return LEVEL_TO_SETPOINT_MAP.get(level);
	}
	
	public static final Setpoint STOW_TEST = new Setpoint(Meters.zero(), Degrees.of(-40), "stow");
	public static final Setpoint INTAKE = new Setpoint(Meters.zero(), Degrees.of(-30), "intake");
	public static final Setpoint ALGAE_PREP_L2 = new Setpoint(Meters.of(0.5), Degrees.of(15), "algae prep L2");
	public static final Setpoint ALGAE_PREP_L3 = new Setpoint(Meters.of(0.88), Degrees.of(15), "algae prep L3");
	public static final Setpoint ALGAE_POP_L2 = new Setpoint(Meters.of(0.45), Degrees.of(15), "algae pop L2");
	public static final Setpoint ALGAE_POP_L3 = new Setpoint(Meters.of(0.83), Degrees.of(15),"algae pop L3");
	
	public static class Stow {
		public static final Angle WRIST_TARGET_1 = Degrees.of(-18);
		// for part 1 of the stow process, we don't care about precision; thus, we make it so that
		// the first pivot angle command stops when the wrist is above this threshold, regardless of precision.
		public static final Angle WRIST_THRESHOLD_1 = WRIST_TARGET_1.plus(Degrees.of(3));
		public static final Distance ELEVATOR_HEIGHT = Meters.zero();
		public static final Distance ELEVATOR_THRESHOLD = Meters.of(0.3);
		public static final Angle WRIST_TARGET_2 = Degrees.of(-40);
	}
	
	public static class Limits {
		// The wrist has to be extended this much to not hit the elevator
		public static final Angle WRIST_LIMIT = Degrees.of(-32);
	}
}
