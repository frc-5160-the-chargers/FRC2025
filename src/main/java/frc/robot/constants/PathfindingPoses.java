package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.FieldConstants.ReefHeight;
import frc.robot.subsystems.swerve.SwerveConfigurator;

import java.util.HashMap;
import java.util.Map;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

public class PathfindingPoses {
	/** How far the robot is offset from the center position of field elements. */
	private static final Translation2d CENTER_POSITION_OFFSET = new Translation2d(
		Inches.of(9), Inches.of(0) // horizontal offset, vertical offset
	);
	private static final Map<Integer, Pose2d> REEF_NODE_POSITIONS_BLUE = new HashMap<>();
	private static Pose2d NORTH_SOURCE_BLUE = Pose2d.kZero;
	private static Pose2d SOUTH_SOURCE_BLUE = Pose2d.kZero;
	
	static {
		initialize(SwerveConfigurator.HARDWARE_SPECS.wheelBase());
	}
	
	/** Adds the appropriate offsets to a pose oriented in the center of a field element. */
	private static Pose2d addOffset(Pose2d initial, Distance wheelBase) {
		double horizontalOffset = CENTER_POSITION_OFFSET.getX();
		double verticalOffset = wheelBase.in(Meters) / 2 + CENTER_POSITION_OFFSET.getY();
		Rotation2d rotation = initial.getRotation();
		return new Pose2d(
			initial.getX() + horizontalOffset * rotation.getCos() + verticalOffset * rotation.getSin(),
			initial.getY() + horizontalOffset * rotation.getSin() + verticalOffset * rotation.getCos(),
			rotation
		);
	}
	
	/** Initializes PathfindingPoses with a different swerve config. */
	public static void initialize(Distance wheelBase) {
		REEF_NODE_POSITIONS_BLUE.clear();
		for (int i = 0; i < 12; i++) {
			// Field constants poses are measured from the branch position itself
			Pose2d nodePose = FieldConstants.Reef.branchPositions.get(i).get(ReefHeight.L1).toPose2d();
			REEF_NODE_POSITIONS_BLUE.put(i, addOffset(nodePose, wheelBase));
		}
		NORTH_SOURCE_BLUE = addOffset(FieldConstants.CoralStation.leftCenterFace, wheelBase);
		SOUTH_SOURCE_BLUE = addOffset(FieldConstants.CoralStation.rightCenterFace, wheelBase);
	}
	
	/**
	 * Returns a blue-alliance pathfinding pose for a scoring position(reef).
	 * @param id The id of the position. From a birds-eye view(open choreo to see this),
	 *           the position of ID 0 is the bottom scoring position on the side of the reef
	 *           facing the driver station(green balls in choreo). All other ids go counterclockwise from there.
	 * @return the scoring position, or Optional.empty() if the position is invalid.
	 */
	public static Pose2d reef(int id) {
		if (id < 0 || id > 11) throw new RuntimeException("Invalid reef id: " + id);
		return REEF_NODE_POSITIONS_BLUE.get(id);
	}
	
	/** Returns the blue alliance pathfinding pose for the north source. */
	public static Pose2d northSource() { return NORTH_SOURCE_BLUE; }
	
	/** Returns the blue alliance pathfinding pose for the south source. */
	public static Pose2d southSource() { return SOUTH_SOURCE_BLUE; }
}
