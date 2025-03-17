package frc.robot.constants;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.chargers.utils.AllianceUtil;
import frc.robot.subsystems.swerve.SwerveDrive.SwerveHardwareSpecs;

import static edu.wpi.first.units.Units.Meters;

/**
 * A struct for storing poses for locations a robot must drive to
 * on the field(reef & source), taking into account offsets of the scoring mechanism.
 */
@Logged
public class TargetPoses {
	/** An enum used to describe which pole(left or right) to align on for a "face" on the hexagonal reef. */
	public enum ReefSide {
		LEFT, RIGHT
	}
	
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
	
	public TargetPoses(
		Translation2d reefOffset,
		Translation2d eastSourceOffset,
		SwerveHardwareSpecs hardwareSpecs
	) {
		this.hardwareSpecs = hardwareSpecs;
		for (int i = 0; i < 12; i++) {
			// Field constants poses are measured from the branch position itself
			Pose2d nodePose = FieldConstants.Reef.branchPositions.get(i).get(FieldConstants.ReefHeight.L1).toPose2d();
			reefBlue[i] = addOffset(nodePose, reefOffset);
		}
		var westSourceOffset = new Translation2d(eastSourceOffset.getX(), -eastSourceOffset.getY());
		eastSourceBlue = addOffset(FieldConstants.CoralStation.leftCenterFace, eastSourceOffset);
		westSourceBlue = addOffset(FieldConstants.CoralStation.rightCenterFace, westSourceOffset);
	}
	
	/**
	 * Determines the best reef pose on your current alliance,
	 * depending on how close the robot is to each position,
	 * as well as the heading difference.
	 */
	public Pose2d closestReefPose(ReefSide side, Pose2d currentBotPose) {
		currentBotPose = AllianceUtil.flipIfRed(currentBotPose);
		var closest = reefBlue[0];
		double smallestTranslationDiff = 1000;
		for (int i = (side == ReefSide.RIGHT ? 0 : 1); i < 12; i += 2) {
			double rotationDiff = Math.abs(currentBotPose.getRotation().getDegrees() - reefBlue[i].getRotation().getDegrees());
			if (rotationDiff > 45) {
				continue;
			}
			double translationDiff = currentBotPose.getTranslation().getDistance(reefBlue[i].getTranslation());
			if (translationDiff < smallestTranslationDiff) {
				smallestTranslationDiff = translationDiff;
				closest = reefBlue[i];
			}
		}
		return AllianceUtil.flipIfRed(closest);
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
