package frc.robot.constants;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import java.util.Map;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static java.util.Map.entry;

/** Represents an elevator + wrist setpoint. */
public record Setpoint(Distance elevatorHeight, Angle wristTarget, String name) {
	private static final Map<Integer, Setpoint> LEVEL_TO_SETPOINT_MAP = Map.ofEntries(
		entry(1, new Setpoint(Meters.zero(), Degrees.zero(), "L1")),
		entry(2, new Setpoint(Meters.of(0.14), Degrees.of(20), "L2")),
		entry(3, new Setpoint(Meters.of(0.52), Degrees.of(20), "L3")),
		entry(4, new Setpoint(Meters.of(1.24), Degrees.of(20), "L4")) // highest possible elevator setpoint
	);
	
	public static final Setpoint STOW_LOW = new Setpoint(Meters.zero(), Degrees.of(-60), "stow low");
	public static final Setpoint STOW_MID = new Setpoint(Meters.of(0.14), Degrees.of(-80), "stow mid");
	public static final Setpoint INTAKE = new Setpoint(Meters.of(0.14), Degrees.of(-30), "intake");
	public static final Setpoint ALGAE_L2 = new Setpoint(Meters.of(0), Degrees.of(0), "algae L2");
	public static final Setpoint ALGAE_L3 = new Setpoint(Meters.of(0), Degrees.of(0), "algae L3");
	
	public static Setpoint score(int level) {
		if (level < 1 || level > 4) {
			throw new RuntimeException("Invalid level: " + level);
		}
		return LEVEL_TO_SETPOINT_MAP.get(level);
	}
}
