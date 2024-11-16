// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package edu.wpi.first.math.geometry;

import java.util.HashMap;
import java.util.Map;

/**
 * A utility to standardize flipping of coordinate data based on the current alliance across
 * different years.
 *
 * <p>If every vendor used this, the user would be able to specify the year and no matter the year
 * the vendor's code is from, the user would be able to flip as expected.
 */
public final class AllianceSymmetry {
	private AllianceSymmetry() {
		throw new UnsupportedOperationException("This is a utility class!");
	}
	
	/** The strategy to use for flipping coordinates over axis of symmetry. */
	public enum SymmetryStrategy {
		/**
		 * X becomes fieldLength - x, leaves the y coordinate unchanged, and heading becomes PI -
		 * heading.
		 */
		VERTICAL {
			@Override
			public double flipX(double x) {
				return activeYear.fieldLength - x;
			}
			
			@Override
			public double flipY(double y) {
				return y;
			}
			
			@Override
			public double flipHeading(double heading) {
				return Math.PI - heading;
			}
		},
		/** X becomes fieldLength - x, Y becomes fieldWidth - y, and heading becomes PI - heading. */
		ROTATIONAL {
			@Override
			public double flipX(double x) {
				return activeYear.fieldLength - x;
			}
			
			@Override
			public double flipY(double y) {
				return activeYear.fieldWidth - y;
			}
			
			@Override
			public double flipHeading(double heading) {
				return Math.PI - heading;
			}
		},
		/**
		 * Leaves the X coordinate unchanged, Y becomes fieldWidth - y, and heading becomes PI -
		 * heading.
		 */
		HORIZONTAL {
			@Override
			public double flipX(double x) {
				return x;
			}
			
			@Override
			public double flipY(double y) {
				return activeYear.fieldWidth - y;
			}
			
			@Override
			public double flipHeading(double heading) {
				return Math.PI - heading;
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
	
	/**
	 * An interface for objects that can be flipped based on the current alliance.
	 *
	 * @param <Self> The type of the object that is flippable.
	 */
	public interface Flippable<Self extends Flippable<Self>> {
		/**
		 * Flips the object based on the supplied {@link SymmetryStrategy} .
		 *
		 * @param strategy The type of symmetry to use for flipping.
		 * @return The flipped object.
		 */
		Self flip(SymmetryStrategy strategy);
		
		/**
		 * Flips the object based on the active flipper.
		 *
		 * @return The flipped object.
		 */
		default Self flip() {
			return flip(getFlipper());
		}
	}
	
	private record YearInfo(SymmetryStrategy flipper, double fieldLength, double fieldWidth) {}
	
	private static final HashMap<Integer, YearInfo> flipperMap =
		new HashMap<Integer, YearInfo>(
			Map.of(
				2022, new YearInfo(SymmetryStrategy.ROTATIONAL, 16.4592, 8.2296),
				2023, new YearInfo(SymmetryStrategy.VERTICAL, 16.54175, 8.0137),
				2024, new YearInfo(SymmetryStrategy.VERTICAL, 16.54175, 8.211)));
	
	private static YearInfo activeYear = flipperMap.get(2024);
	
	/**
	 * Get the flipper that is currently active for flipping coordinates. It's reccomended not to
	 * store this locally as the flipper may change.
	 *
	 * @return The active flipper.
	 */
	public static SymmetryStrategy getFlipper() {
		return activeYear.flipper;
	}
	
	/**
	 * Set the year to determine the Alliance Coordinate Flipper to use.
	 *
	 * @param year The year to set the flipper to. [2022 - 2024]
	 */
	public static void setYear(int year) {
		if (!flipperMap.containsKey(year)) {
			// Throw an exception instead of just reporting an error
			// because not flipping correctly during an auto routine
			// could cause a robot to damage itself or others.
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
	 * Flips the {@link Flippable} object.
	 *
	 * @param <T> The type of the object to flip.
	 * @param flippable The object to flip.
	 * @return The flipped object.
	 */
	public static <T extends Flippable<T>> T flip(Flippable<T> flippable) {
		return flippable.flip();
	}
}

