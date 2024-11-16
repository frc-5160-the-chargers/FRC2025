package frc.chargers.utils;


import static frc.chargers.utils.DoubleEquivalency.equivalent;

public record ChassisPowers(double xPower, double yPower, double zPower) {
	@Override
	public boolean equals(Object other) {
		return other instanceof ChassisPowers casted &&
			   equivalent(xPower(), casted.xPower()) &&
			   equivalent(yPower(), casted.yPower()) &&
			   equivalent(zPower(), casted.zPower());
	}
}
