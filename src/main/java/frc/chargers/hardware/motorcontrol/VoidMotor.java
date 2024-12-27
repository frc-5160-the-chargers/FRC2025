package frc.chargers.hardware.motorcontrol;

import com.pathplanner.lib.config.PIDConstants;
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
	public void setVoltage(double volts) {}
	public void setVelocity(double velocityRadPerSec, double ffVolts) {}
	public void moveToPosition(double positionRads, double ffVolts) {}
	public void setTorqueCurrent(double currentAmps) {}
	public void setCoastMode(boolean on) {}
	public void setPositionPID(PIDConstants constants) {}
	public void setVelocityPID(PIDConstants constants) {}
	public void enableContinuousInput() {}
}