package frc.robot.positions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import lombok.RequiredArgsConstructor;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

@RequiredArgsConstructor
public enum IntakePosition {
	NORTH_SOURCE(Pose2d.kZero),
	SOUTH_SOURCE(Pose2d.kZero);
	
	public final Pose2d pose;
	// TODO
	public static final Distance ELEVATOR_HEIGHT = Meters.of(0);
	public static final Angle PIVOT_ANGLE = Radians.of(0);
}
