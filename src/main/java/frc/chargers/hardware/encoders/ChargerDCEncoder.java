package frc.chargers.hardware.encoders;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import static edu.wpi.first.math.util.Units.rotationsToRadians;

/**
 * A wrapper around a DutyCycleEncoder.
 */
public class ChargerDCEncoder implements Encoder {
	private final boolean inverted;
	private final DutyCycleEncoder encoder;
	private double offset = 0.0;
	
	public ChargerDCEncoder(int port, boolean inverted) {
		this.encoder = new DutyCycleEncoder(port);
		this.inverted = inverted;
	}
	
	@Override
	public double positionRad() {
		return rotationsToRadians(encoder.get() - offset) * (inverted ? -1 : 1);
	}
	
	@Override
	public double velocityRadPerSec() { return 0; }
	
	@Override
	public void setPositionReading(Angle angle) { offset = encoder.get();  }
}
