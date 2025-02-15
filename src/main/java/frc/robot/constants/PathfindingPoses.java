package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.SwerveDrive.SwerveHardwareSpecs;

import java.util.HashMap;
import java.util.Map;

import static edu.wpi.first.units.Units.Meters;

/** A struct for storing pathfinding poses related to the robot. */
public class PathfindingPoses {
	private final SwerveHardwareSpecs hardwareSpecs;
	private final Map<Integer, Pose2d> reefPositionsBlue = new HashMap<>();
	private final Pose2d northSourcePositionBlue;
	private final Pose2d southSourcePositionBlue;
	
	public PathfindingPoses(
		Translation2d reefOffset,
		Translation2d sourceOffset,
		SwerveHardwareSpecs hardwareSpecs
	) {
		this.hardwareSpecs = hardwareSpecs;
		for (int i = 0; i < 12; i++) {
			// Field constants poses are measured from the branch position itself
			Pose2d nodePose = FieldConstants.Reef.branchPositions.get(i).get(FieldConstants.ReefHeight.L1).toPose2d();
			reefPositionsBlue.put(i, addOffset(nodePose, reefOffset));
		}
		northSourcePositionBlue = addOffset(FieldConstants.CoralStation.leftCenterFace, sourceOffset);
		southSourcePositionBlue = addOffset(FieldConstants.CoralStation.rightCenterFace, sourceOffset);
	}
	
	/** Adds the appropriate offsets to a pose oriented in the center of a field element. */
	private Pose2d addOffset(Pose2d initial, Translation2d offset) {
		double horizontalOffset = offset.getX();
		double verticalOffset = hardwareSpecs.wheelBase().in(Meters) / 2
			                        + hardwareSpecs.widthOfBumpers().in(Meters)
			                        + offset.getY();
		Rotation2d rotation = initial.getRotation();
		return new Pose2d(
			initial.getX() + verticalOffset * rotation.getCos() + horizontalOffset * rotation.getSin(),
			initial.getY() + verticalOffset * rotation.getSin() + horizontalOffset * rotation.getCos(),
			rotation.plus(Rotation2d.k180deg)
		);
	}
	
	/**
	 * Returns a blue-alliance pathfinding pose for a scoring position(reef).
	 * @param id The id of the position. From a birds-eye view(open choreo to see this),
	 *           the position of ID 0 is the bottom scoring position on the side of the reef
	 *           facing the driver station(green balls in choreo). All other ids go counterclockwise from there.
	 * @return the scoring position, or Optional.empty() if the position is invalid.
	 */
	public Pose2d reef(int id) {
		if (id < 0 || id > 11) throw new RuntimeException("Invalid reef id: " + id);
		return reefPositionsBlue.get(id);
	}
	
	/** Returns the blue alliance pathfinding pose for the north source. */
	public Pose2d northSource() { return northSourcePositionBlue; }
	
	/** Returns the blue alliance pathfinding pose for the south source. */
	public Pose2d southSource() { return southSourcePositionBlue; }
}
