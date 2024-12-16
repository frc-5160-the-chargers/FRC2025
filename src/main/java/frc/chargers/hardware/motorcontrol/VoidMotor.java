package frc.chargers.hardware.motorcontrol;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.units.measure.Angle;
import frc.chargers.hardware.encoders.Encoder;

/**
 * An null motor.
 */
class VoidMotor implements Motor {
	private static final Encoder encoder = new Encoder() {
		public double positionRad() { return 0; }
		public double velocityRadPerSec() { return 0; }
		public void setPositionReading(Angle angle) {}
	};
	public Encoder encoder() { return encoder; }
	public double outputVoltage() { return 0; }
	public double statorCurrent() { return 0; }
	public double tempCelsius() { return 0; }
	public void setVoltage(double volts) {}
	public void setVelocity(double velocityRadPerSec, double ffVolts) {}
	public void moveToPosition(double positionRads, double ffVolts) {}
	public void setTorqueCurrent(double currentAmps) {}
	public void setCoastMode(boolean on) {}
	public void setPositionPID(PIDConstants constants) {}
	public void setVelocityPID(PIDConstants constants) {}
	public void enableContinuousInput() {}
}