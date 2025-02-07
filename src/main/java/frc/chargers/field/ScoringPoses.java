package frc.chargers.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.chargers.field.FieldConstants.ReefHeight;
import frc.robot.subsystems.swerve.SwerveConfigurator;
import frc.robot.subsystems.swerve.SwerveDrive;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

public class ScoringPoses {
	private static final Translation2d ROBOT_RELATIVE_TRANSLATION = new Translation2d(
		Inches.of(9), Inches.of(0) // x, y
	);
	private static final Map<Integer, Pose2d> SCORE_POSITION_MAP = new HashMap<>();
	
	static {
		initialize(SwerveConfigurator.DEFAULT_DRIVE_CONFIG);
	}
	
	public static void initialize(SwerveDrive.SwerveDriveConfig config) {
		SCORE_POSITION_MAP.clear();
		double horizontalOffset = config.ofHardware().trackWidth().in(Meters) / 2 + ROBOT_RELATIVE_TRANSLATION.getX();
		double verticalOffset = ROBOT_RELATIVE_TRANSLATION.getY();
		for (int i = 0; i < 12; i++) {
			// Field constants poses are measured from the branch position itself
			Pose2d nodePose = FieldConstants.Reef.branchPositions.get(i).get(ReefHeight.L1).toPose2d();
			Rotation2d rotation = nodePose.getRotation();
			Pose2d robotPose = new Pose2d(
				nodePose.getX() + horizontalOffset * rotation.getCos() + verticalOffset * rotation.getSin(),
				nodePose.getY() + horizontalOffset * rotation.getSin() + verticalOffset * rotation.getCos(),
				rotation
			);
			SCORE_POSITION_MAP.put(i, robotPose);
		}
	}
	
	/**
	 * Gets the proper pathfinding position the robot should target from an id.
	 * @param id The id of the position. From a birds-eye view(open choreo to see this),
	 *           the position of ID 0 is the bottom scoring position on the side of the reef
	 *           facing the driver station(green balls in choreo). All other ids go counterclockwise from there.
	 * @return the scoring position, or Optional.empty() if the position is invalid.
	 */
	public static Optional<Pose2d> getPose(int id) {
		return Optional.ofNullable(SCORE_POSITION_MAP.get(id));
	}
}
