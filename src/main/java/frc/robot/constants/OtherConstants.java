package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

/** Constants that aren't specific to any subsystem. */
public class OtherConstants {
	private OtherConstants() {}
	
	public static final double NUDGE_OUTPUT = 0.03;
	// to the right
	public static final Distance INTAKE_OFFSET_FROM_CENTER = Inches.of(1.1875);
	public static final Translation2d REEF_SCORE_OFFSET =
		new Translation2d(Inches.of(-7.5), INTAKE_OFFSET_FROM_CENTER);
	public static final Translation2d SOURCE_OFFSET =
		new Translation2d(Meters.of(-0.1), INTAKE_OFFSET_FROM_CENTER.plus(Inches.of(-20)));
}
