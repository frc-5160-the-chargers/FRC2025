package frc.chargers.utils;

import static frc.chargers.utils.UtilMethods.equivalent;

/**
 * A struct that represents percentages
 * @param xPower
 * @param yPower
 * @param rotationPower
 */
public record ChassisPowers(double xPower, double yPower, double rotationPower) {
	
	@Override
	public boolean equals(Object other) {
		return other instanceof ChassisPowers casted &&
			   equivalent(xPower(), casted.xPower()) &&
			   equivalent(yPower(), casted.yPower()) &&
			   equivalent(rotationPower(), casted.rotationPower());
	}
}
