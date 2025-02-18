package frc.robot.constants;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.SwerveDrive.SwerveHardwareSpecs;

import static edu.wpi.first.units.Units.Meters;

/** A struct for storing pathfinding poses related to the robot. */
@Logged
public class PathfindingPoses {
	@NotLogged private final SwerveHardwareSpecs hardwareSpecs;
	/**
	 * A list of pathfinding poses corresponding to the various scoring locations
	 * the robot should target on the blue alliance side.
	 * The pose at index 0 is the bottom scoring position on the side of the reef
	 * facing the driver station(green balls in choreo). The order is clockwise positive.
	 */
	public final Pose2d[] reefBlue = new Pose2d[12];
	/** The north source intaking position on the blue alliance side. */
	public final Pose2d eastSourceBlue;
	/** The south source intaking position on the blue alliance side. */
	public final Pose2d westSourceBlue;
	
	public PathfindingPoses(
		Translation2d reefOffset,
		Translation2d eastSourceOffset,
		Translation2d westSourceOffset,
		SwerveHardwareSpecs hardwareSpecs
	) {
		this.hardwareSpecs = hardwareSpecs;
		for (int i = 0; i < 12; i++) {
			// Field constants poses are measured from the branch position itself
			Pose2d nodePose = FieldConstants.Reef.branchPositions.get(i).get(FieldConstants.ReefHeight.L1).toPose2d();
			reefBlue[i] = addOffset(nodePose, reefOffset);
		}
		eastSourceBlue = addOffset(FieldConstants.CoralStation.leftCenterFace, eastSourceOffset);
		westSourceBlue = addOffset(FieldConstants.CoralStation.rightCenterFace, westSourceOffset);
	}
	
	/** Adds the appropriate offsets to a pose oriented in the center of a field element. */
	private Pose2d addOffset(Pose2d initial, Translation2d offset) {
		double horizontalOffset = offset.getY();
		double verticalOffset = - hardwareSpecs.wheelBase().in(Meters) / 2
			                        - hardwareSpecs.widthOfBumpers().in(Meters)
			                        + offset.getX();
		Rotation2d rotation = initial.getRotation().plus(Rotation2d.k180deg);
		return new Pose2d(
			initial.getTranslation().plus(new Translation2d(verticalOffset, horizontalOffset).rotateBy(rotation)),
			rotation
		);
	}
}
