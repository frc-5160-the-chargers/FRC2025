package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.*;

/** Constants that aren't specific to any subsystem. */
public class OtherConstants {
	private OtherConstants() {}
	
	public static final boolean USE_PATHFINDING = true;
	public static final double NUDGE_OUTPUT = 0.07;
	public static final boolean IS_DANIEL_COMPUTER = true;
	// to the right
	public static final Distance INTAKE_OFFSET_FROM_CENTER = Inches.of(1.1875);
	public static final Translation2d REEF_SCORE_OFFSET =
		new Translation2d(Inches.of(-12.5), INTAKE_OFFSET_FROM_CENTER);
	public static final Translation2d SOURCE_OFFSET =
		new Translation2d(Meters.of(-0.1), INTAKE_OFFSET_FROM_CENTER.plus(Inches.of(-20)));
	public static final Angle SAFE_WRIST_ANGLE = Degrees.of(-37); // When the wrist is past this position, the elevator can move safely
}
