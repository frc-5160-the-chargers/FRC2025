package frc.chargers.hardware.encoders;

import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Radians;

/**
 * An encoder that does nothing.
 */
public class VoidEncoder implements Encoder {
	private double positionTarget = 0;
	private double velocityTarget = 0;
	
	@Override
	public double positionRad() {
		return positionTarget;
	}
	
	@Override
	public double velocityRadPerSec() {
		return velocityTarget;
	}
	
	@Override
	public void setPositionReading(Angle angle) {
		positionTarget = angle.in(Radians);
	}
	
	public void setVelocityReading(double velocityRadPerSec) {
		velocityTarget = velocityRadPerSec;
	}
}
