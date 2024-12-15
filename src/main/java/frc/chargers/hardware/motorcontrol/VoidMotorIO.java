package frc.chargers.hardware.motorcontrol;

import edu.wpi.first.units.measure.Angle;
import frc.chargers.hardware.encoders.EncoderIO;

/**
 * An implementation of a motor IO that does nothing.
 */
class VoidMotorIO implements MotorIO {
	private static final EncoderIO encoder = new EncoderIO() {
		public double positionRad() { return 0; }
		public double velocityRadPerSec() { return 0; }
		public void setPositionReading(Angle angle) {}
	};
	public EncoderIO encoder() { return encoder; }
	public double outputVoltage() { return 0; }
	public double statorCurrent() { return 0; }
	public double tempCelsius() { return 0; }
	public void setVoltage(double volts) {}
	public void setVelocity(double velocityRadPerSec, double ffVolts) {}
	public void moveToPosition(double positionRads, double ffVolts) {}
	public void setTorqueCurrent(double currentAmps) {}
	public void setCoastMode(boolean on) {}
	public void setPositionPID(double p, double i, double d) {}
	public void setVelocityPID(double p, double i, double d) {}
	public void enableContinuousInput() {}
}