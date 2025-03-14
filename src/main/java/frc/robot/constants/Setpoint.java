package frc.robot.constants;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import java.util.Map;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static java.util.Map.entry;

/** Represents an elevator + wrist setpoint. */
public record Setpoint(Distance elevatorHeight, Angle wristTarget, boolean isParallel, String name) {
	private static final Map<Integer, Setpoint> LEVEL_TO_SETPOINT_MAP = Map.ofEntries(
		entry(1, new Setpoint(Meters.zero(), Degrees.zero(), true, "L1")),
		entry(2, new Setpoint(Meters.of(0.13), Degrees.of(10), true, "L2")),
		entry(3, new Setpoint(Meters.of(0.53), Degrees.of(10), true, "L3")),
		entry(4, new Setpoint(Meters.of(1.27), Degrees.of(22), true, "L4")) // highest possible elevator setpoint
	);
	
	public static final Setpoint STOW_LOW = new Setpoint(Meters.zero(), Degrees.of(-30), false, "stow low");
	public static final Setpoint STOW_MID = new Setpoint(Meters.of(0), Degrees.of(-30), false, "stow mid");
	public static final Setpoint INTAKE = new Setpoint(Meters.of(0), Degrees.of(-30), true, "intake");
	
	public static final Setpoint ALGAE_PREP_L2 = new Setpoint(Meters.of(0.5), Degrees.of(15), true, "algae prep L2");
	public static final Setpoint ALGAE_PREP_L3 = new Setpoint(Meters.of(0.88), Degrees.of(15), true, "algae prep L3");
	public static final Setpoint ALGAE_POP_L2 = new Setpoint(Meters.of(0.4), Degrees.of(15), true, "algae pop L2");
	public static final Setpoint ALGAE_POP_L3 = new Setpoint(Meters.of(0.78), Degrees.of(15), true,"algae pop L3");
	
	public static Setpoint score(int level) {
		if (level < 1 || level > 4) {
			throw new RuntimeException("Invalid level: " + level);
		}
		return LEVEL_TO_SETPOINT_MAP.get(level);
	}
}
