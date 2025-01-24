package frc.chargers.hardware.motorcontrol;

import edu.wpi.first.epilogue.Logged;
import frc.chargers.hardware.encoders.Encoder;
import frc.chargers.utils.PIDConstants;
import lombok.With;

/**
 * A type that represents a modern motor controller.
 */
@Logged
public interface Motor extends AutoCloseable {
	// provides a "withX()" method for every property -
	// config.withGearRatio(double), config.withPositionPID(constants), etc.
	@With
	record ControlsConfig(
		double gearRatio,
		PIDConstants positionPID,
		PIDConstants velocityPID,
		boolean continuousInput
	){
		public static final ControlsConfig EMPTY =
			new ControlsConfig(1.0, PIDConstants.VOID, PIDConstants.VOID, false);
	}
	
	Encoder encoder();
	double outputVoltage();
	double statorCurrent();
	double tempCelsius();
	
	void setControlsConfig(ControlsConfig newConfig);
	void setVoltage(double volts);
	void setVelocity(double velocityRadPerSec, double ffVolts);
	void moveToPosition(double positionRads, double ffVolts);
	
	default void moveToPosition(double angleRads) {
		moveToPosition(angleRads, 0);
	}
	default double supplyCurrent() {
		return statorCurrent() * outputVoltage() / 12.0;
	}
	default double torqueCurrent() { return 0.0; }
	default void setTorqueCurrent(double currentAmps) {}
	@Override default void close() {}
}
