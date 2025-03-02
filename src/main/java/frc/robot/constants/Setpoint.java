package frc.robot.constants;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

import java.util.Map;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.util.struct.StructGenerator.genRecord;
import static java.util.Map.entry;

/** Represents an elevator + wrist setpoint. */
public record Setpoint(Distance elevatorHeight, Angle wristTarget) implements StructSerializable {
	public static final Struct<Setpoint> struct = genRecord(Setpoint.class);
	
	private static final Map<Integer, Setpoint> LEVEL_TO_SETPOINT_MAP = Map.ofEntries(
		entry(1, new Setpoint(Meters.zero(), Degrees.zero())),
		entry(2, new Setpoint(Meters.of(0.34), Degrees.of(30))),
		entry(3, new Setpoint(Meters.of(0.74), Degrees.of(30))),
		entry(4, new Setpoint(Meters.of(1.2), Degrees.of(15)))
	);
	
	public static final Setpoint STOW_LOW = new Setpoint(Meters.zero(), Degrees.of(-80));
	public static final Setpoint STOW_MID = new Setpoint(Meters.of(0.14), Degrees.of(-80));
	public static final Setpoint INTAKE = new Setpoint(Meters.of(0.14), Degrees.of(-30));
	
	public static Setpoint score(int level) {
		if (level < 1 || level > 4) {
			throw new RuntimeException("Invalid level: " + level);
		}
		return LEVEL_TO_SETPOINT_MAP.get(level);
	}
}
