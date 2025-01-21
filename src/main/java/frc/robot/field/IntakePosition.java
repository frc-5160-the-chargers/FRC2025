package frc.robot.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

public enum IntakePosition {
	NORTH_SOURCE(Pose2d.kZero),
	SOUTH_SOURCE(Pose2d.kZero);
	
	public final Pose2d pose;
	IntakePosition(Pose2d pose) { this.pose = pose; }
	// TODO
	public static final Distance ELEVATOR_HEIGHT = Meters.of(0);
	public static final Angle PIVOT_ANGLE = Radians.of(0);
}
