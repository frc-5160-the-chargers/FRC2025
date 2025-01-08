package frc.chargers.hardware.motorcontrol;

import frc.chargers.hardware.encoders.Encoder;
import frc.chargers.hardware.encoders.VoidEncoder;

/**
 * An null motor.
 */
public class VoidMotor implements Motor {
	private static final Encoder ENCODER = new VoidEncoder();
	
	public Encoder encoder() { return ENCODER; }
	public double outputVoltage() { return 0; }
	public double statorCurrent() { return 0; }
	public double tempCelsius() { return 0; }
	public void setCommonConfig(CommonConfig newConfig) {}
	public void setVoltage(double volts) {}
	public void setVelocity(double velocityRadPerSec, double ffVolts) {}
	public void moveToPosition(double positionRads, double ffVolts) {}
	public void setTorqueCurrent(double currentAmps) {}
}