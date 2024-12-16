package frc.chargers.hardware.encoders;

import edu.wpi.first.units.measure.Angle;

/**
 * Represents a null encoder.
 */
public class VoidEncoder implements Encoder {
	@Override public double positionRad() { return 0; }
	@Override public double velocityRadPerSec() { return 0; }
	@Override public void setPositionReading(Angle angle) {}
}
