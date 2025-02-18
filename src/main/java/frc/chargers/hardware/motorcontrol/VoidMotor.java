package frc.chargers.hardware.motorcontrol;

import frc.chargers.hardware.encoders.Encoder;
import frc.chargers.hardware.encoders.VoidEncoder;

/**
 * A motor that does nothing.
 */
public class VoidMotor implements Motor {
	private static final Encoder ENCODER = new VoidEncoder();
	
	public Encoder encoder() { return ENCODER; }
	public double outputVoltage() { return -1; }
	public double statorCurrent() { return -1; }
	public double tempCelsius() { return -1; }
	public int id() { return -1; }
	public void setControlsConfig(ControlsConfig newConfig) {}
	public void setVoltage(double volts) {}
	public void setVelocity(double velocityRadPerSec, double ffVolts) {}
	public void moveToPosition(double positionRads, double ffVolts) {}
	public void setCoastMode(boolean enabled) {}
}