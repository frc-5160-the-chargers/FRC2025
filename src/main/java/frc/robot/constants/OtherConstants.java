package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

/** Constants that aren't specific to any subsystem. */
public class OtherConstants {
	private OtherConstants() {}
	
	public static final boolean USE_PATHFINDING = true;
	
	public static final Translation2d REEF_SCORE_OFFSET =
		new Translation2d(Inches.of(-12.5), Inches.of(-7));
	public static final Translation2d SOURCE_OFFSET =
		new Translation2d(Meters.of(-0.13), Meters.of(-0.5));
	
	public static final int DRIVER_CONTROLLER_PORT = 0;
	public static final int MANUAL_CONTROLLER_PORT = 1;
}
