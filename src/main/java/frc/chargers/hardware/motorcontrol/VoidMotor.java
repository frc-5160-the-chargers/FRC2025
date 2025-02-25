package frc.chargers.hardware.motorcontrol;

import frc.chargers.hardware.encoders.Encoder;
import frc.chargers.hardware.encoders.VoidEncoder;

import static edu.wpi.first.units.Units.Radians;

/**
 * A motor that does nothing.
 */
public class VoidMotor implements Motor {
	private final VoidEncoder encoder = new VoidEncoder();
	
	public Encoder encoder() { return encoder; }
	public double outputVoltage() { return -1; }
	public double statorCurrent() { return -1; }
	public double tempCelsius() { return -1; }
	public int id() { return -1; }
	public void setControlsConfig(ControlsConfig newConfig) {}
	public void setVoltage(double volts) {}
	public void setVelocity(double velocityRadPerSec, double ffVolts) {
		encoder.setVelocityReading(velocityRadPerSec);
	}
	public void moveToPosition(double positionRads, double ffVolts) {
		encoder.setPositionReading(Radians.of(positionRads));
	}
	public void setCoastMode(boolean enabled) {}
}