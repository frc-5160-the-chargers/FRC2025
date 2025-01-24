package frc.chargers.hardware.encoders;

import edu.wpi.first.units.measure.Angle;

/**
 * An encoder that returns nothing.
 */
public class VoidEncoder implements Encoder {
	public double positionRad() { return 0; }
	public double velocityRadPerSec() { return 0; }
	public void setPositionReading(Angle angle) {}
}
