package frc.chargers.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.HashMap;

/**
 * A utility to standardize flipping of coordinate data based on the current alliance across
 * different years.
 *
 * <p>The corresponding flip() methods will always flip to the other side, regardless of alliance.
 * On the other hand, the flipIfRed() methods will only flip poses/rotations to the blue side of the field.
 *
 * <p>If every vendor used this, the user would be able to specify the year and no matter the year
 * the vendor's code is from, the user would be able to flip as expected.
 *
 * <p>This API still allows vendors and users to match case against the flipping variant as a way to
 * specially handle cases or throw errors if a variant is explicitly not supported.
 */
public class AllianceUtil {
	/** The flipper to use for flipping coordinates. */
	public enum Flipper {
		/**
		 * X becomes fieldLength - x, leaves the y coordinate unchanged, and heading becomes PI -
		 * heading.
		 */
		MIRRORED {
			public double flipX(double x) {
				return activeYear.fieldLength - x;
			}
			
			public double flipY(double y) {
				return y;
			}
			
			public double flipHeading(double heading) {
				return Math.PI - heading;
			}
		},
		/** X becomes fieldLength - x, Y becomes fieldWidth - y, and heading becomes PI - heading. */
		ROTATE_AROUND {
			public double flipX(double x) {
				return activeYear.fieldLength - x;
			}
			
			public double flipY(double y) {
				return activeYear.fieldWidth - y;
			}
			
			public double flipHeading(double heading) {
				return Math.PI + heading;
			}
		};
		
		/**
		 * Flips the X coordinate.
		 *
		 * @param x The X coordinate to flip.
		 * @return The flipped X coordinate.
		 */
		public abstract double flipX(double x);
		
		/**
		 * Flips the Y coordinate.
		 *
		 * @param y The Y coordinate to flip.
		 * @return The flipped Y coordinate.
		 */
		public abstract double flipY(double y);
		
		/**
		 * Flips the heading.
		 *
		 * @param heading The heading to flip.
		 * @return The flipped heading.
		 */
		public abstract double flipHeading(double heading);
	}
	
	private record YearInfo(Flipper flipper, double fieldLength, double fieldWidth) {}
	
	private static final HashMap<Integer, YearInfo> flipperMap =
		new HashMap<>() {
			{
				put(2024, new YearInfo(Flipper.MIRRORED, 16.5811, 8.19912));
				put(2025, new YearInfo(Flipper.ROTATE_AROUND, 17.548, 8.052));
			}
		};
	
	private static YearInfo activeYear = flipperMap.get(2025);
	
	/** Default constructor. */
	private AllianceUtil() {}
	
	/**
	 * Get the flipper that is currently active for flipping coordinates. It's recommended not to
	 * store this locally as the flipper may change.
	 *
	 * @return The active flipper.
	 */
	public static Flipper getFlipper() {
		return activeYear.flipper;
	}
	
	/**
	 * Returns if you are on red alliance.
	 *
	 * @return If you are on red alliance.
	 */
	public static boolean shouldFlip() {
		return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
	}
	
	/**
	 * Set the year to determine the Alliance Coordinate Flipper to use.
	 *
	 * @param year The year to set the flipper to. [2020 - 2024]
	 */
	public static void setYear(int year) {
		if (!flipperMap.containsKey(year)) {
			throw new IllegalArgumentException("Year " + year + " is not supported.");
		}
		activeYear = flipperMap.get(year);
	}
	
	/**
	 * Flips the X coordinate.
	 *
	 * @param x The X coordinate to flip.
	 * @return The flipped X coordinate.
	 */
	public static double flipX(double x) {
		return activeYear.flipper.flipX(x);
	}
	
	/**
	 * Flips the Y coordinate.
	 *
	 * @param y The Y coordinate to flip.
	 * @return The flipped Y coordinate.
	 */
	public static double flipY(double y) {
		return activeYear.flipper.flipY(y);
	}
	
	/**
	 * Flips the heading.
	 *
	 * @param heading The heading to flip.
	 * @return The flipped heading.
	 */
	public static double flipHeading(double heading) {
		return activeYear.flipper.flipHeading(heading);
	}
	
	/**
	 * Flips the translation.
	 *
	 * @param translation The translation to flip.
	 * @return The flipped translation.
	 */
	public static Translation2d flip(Translation2d translation) {
		return new Translation2d(flipX(translation.getX()), flipY(translation.getY()));
	}
	
	/**
	 * Flips the rotation.
	 *
	 * @param rotation The rotation to flip.
	 * @return The flipped rotation.
	 */
	public static Rotation2d flip(Rotation2d rotation) {
		return switch (activeYear.flipper) {
			case MIRRORED -> new Rotation2d(-rotation.getCos(), rotation.getSin());
			case ROTATE_AROUND -> new Rotation2d(-rotation.getCos(), -rotation.getSin());
		};
	}
	
	/**
	 * Flips the pose.
	 *
	 * @param pose The pose to flip.
	 * @return The flipped pose.
	 */
	public static Pose2d flip(Pose2d pose) {
		return new Pose2d(flip(pose.getTranslation()), flip(pose.getRotation()));
	}
	
	/**
	 * Flips the translation.
	 *
	 * @param translation The translation to flip.
	 * @return The flipped translation.
	 */
	public static Translation3d flip(Translation3d translation) {
		return new Translation3d(
			flipX(translation.getX()), flipY(translation.getY()), translation.getZ());
	}
	
	/**
	 * Flips the rotation.
	 *
	 * @param rotation The rotation to flip.
	 * @return The flipped rotation.
	 */
	public static Rotation3d flip(Rotation3d rotation) {
		return new Rotation3d(
			rotation.getX(), rotation.getY(), flip(rotation.toRotation2d()).getRadians());
	}
	
	/**
	 * Flips the pose.
	 *
	 * @param pose The pose to flip.
	 * @return The flipped pose.
	 */
	public static Pose3d flip(Pose3d pose) {
		return new Pose3d(flip(pose.getTranslation()), flip(pose.getRotation()));
	}
	
	/**
	 * Flips the X coordinate if on red alliance.
	 *
	 * @param x The X coordinate to flip.
	 * @return The flipped X coordinate.
	 */
	public static double flipXIfRed(double x) {
		if (!isRed()) return x;
		return activeYear.flipper.flipX(x);
	}
	
	/**
	 * Flips the Y coordinate if on red alliance.
	 *
	 * @param y The Y coordinate to flip.
	 * @return The flipped Y coordinate.
	 */
	public static double flipYIfRed(double y) {
		if (!isRed()) return y;
		return activeYear.flipper.flipY(y);
	}
	
	/**
	 * Flips the heading if on red alliance.
	 *
	 * @param heading The heading to flip.
	 * @return The flipped heading.
	 */
	public static double flipHeadingIfRed(double heading) {
		if (!isRed()) return heading;
		return activeYear.flipper.flipHeading(heading);
	}
	
	/**
	 * Flips the translation if on red alliance.
	 *
	 * @param translation The translation to flip.
	 * @return The flipped translation.
	 */
	public static Translation2d flipIfRed(Translation2d translation) {
		if (!isRed()) return translation;
		return new Translation2d(flipX(translation.getX()), flipY(translation.getY()));
	}
	
	/**
	 * Flips the rotation if on red alliance.
	 *
	 * @param rotation The rotation to flip.
	 * @return The flipped rotation.
	 */
	public static Rotation2d flipIfRed(Rotation2d rotation) {
		if (!isRed()) return rotation;
		return switch (activeYear.flipper) {
			case MIRRORED -> new Rotation2d(-rotation.getCos(), rotation.getSin());
			case ROTATE_AROUND -> new Rotation2d(-rotation.getCos(), -rotation.getSin());
		};
	}
	
	/**
	 * Flips the pose if on red alliance.
	 *
	 * @param pose The pose to flip.
	 * @return The flipped pose.
	 */
	public static Pose2d flipIfRed(Pose2d pose) {
		if (!isRed()) return pose;
		return new Pose2d(flip(pose.getTranslation()), flip(pose.getRotation()));
	}
	
	/**
	 * Flips the translation if on red alliance.
	 *
	 * @param translation The translation to flip.
	 * @return The flipped translation.
	 */
	public static Translation3d flipIfRed(Translation3d translation) {
		if (!isRed()) return translation;
		return new Translation3d(
			flipX(translation.getX()), flipY(translation.getY()), translation.getZ());
	}
	
	/**
	 * Flips the rotation if on red alliance.
	 *
	 * @param rotation The rotation to flip.
	 * @return The flipped rotation.
	 */
	public static Rotation3d flipIfRed(Rotation3d rotation) {
		if (!isRed()) return rotation;
		return new Rotation3d(
			rotation.getX(), rotation.getY(), flip(rotation.toRotation2d()).getRadians());
	}
	
	/**
	 * Flips the pose if on red alliance.
	 *
	 * @param pose The pose to flip.
	 * @return The flipped pose.
	 */
	public static Pose3d flipIfRed(Pose3d pose) {
		if (!isRed()) return pose;
		return new Pose3d(flip(pose.getTranslation()), flip(pose.getRotation()));
	}
	
	public static boolean isBlue() {
		return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
	}
	
	public static boolean isRed() {
		return !isBlue();
	}
}
