package frc.chargers.field;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;

import java.util.HashMap;
import java.util.Map;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.wpilibj.Alert.AlertType.kError;

public record ScoringLevel(int value) {
	private static final Map<Integer, Distance> LEVEL_TO_ELEVATOR_HEIGHT_MAP = new HashMap<>();
	private static final Map<Integer, Angle> LEVEL_TO_PIVOT_ANGLE_MAP = new HashMap<>();
	private static final Distance DEFAULT_ELEVATOR_HEIGHT = Meters.of(0);
	private static final Angle DEFAULT_PIVOT_ANGLE = Radians.of(0);
	
	static {
		// TODO fill these out
		LEVEL_TO_ELEVATOR_HEIGHT_MAP.put(1, Meters.of(0));
		LEVEL_TO_ELEVATOR_HEIGHT_MAP.put(2, Meters.of(1));
		LEVEL_TO_ELEVATOR_HEIGHT_MAP.put(3, Meters.of(2));
		LEVEL_TO_ELEVATOR_HEIGHT_MAP.put(4, Meters.of(3));
		
		LEVEL_TO_PIVOT_ANGLE_MAP.put(1, Radians.of(0));
		LEVEL_TO_PIVOT_ANGLE_MAP.put(2, Radians.of(1));
		LEVEL_TO_PIVOT_ANGLE_MAP.put(3, Radians.of(2));
		LEVEL_TO_PIVOT_ANGLE_MAP.put(4, Radians.of(3));
	}
	
	public ScoringLevel {
		if (value < 1 || value > 4) {
			new Alert("Invalid scoring level: " + value, kError).set(true);
		}
	}
	
	public Distance elevatorHeight() {
		return LEVEL_TO_ELEVATOR_HEIGHT_MAP.getOrDefault(value, DEFAULT_ELEVATOR_HEIGHT);
	}
	
	public Angle pivotAngle() {
		return LEVEL_TO_PIVOT_ANGLE_MAP.getOrDefault(value, DEFAULT_PIVOT_ANGLE);
	}
}
