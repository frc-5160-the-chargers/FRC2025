package frc.chargers.hardware.motorcontrol;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.chargers.hardware.encoders.Encoder;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

@Logged
public interface Motor {
	Encoder encoder();
	double outputVoltage();
	double statorCurrent();
	double tempCelsius();
	
	void setVoltage(double volts);
	void setVelocity(double velocityRadPerSec, double ffVolts);
	void moveToPosition(double positionRads, double ffVolts);
	void setTorqueCurrent(double currentAmps);
	void setCoastMode(boolean on);
	
	void setPositionPID(PIDConstants constants);
	void setVelocityPID(PIDConstants constants);
	void enableContinuousInput();
	
	default void setVelocity(AngularVelocity velocity, double ffVolts) {
		setVelocity(velocity.in(RadiansPerSecond), ffVolts);
	}
	
	default void moveToPosition(Angle position, double ffVolts) {
		moveToPosition(position.in(Radians), ffVolts);
	}
	default void moveToPosition(double angleRads) {
		moveToPosition(angleRads, 0);
	}
	default void moveToPosition(Angle angle) {
		moveToPosition(angle.in(Radians), 0);
	}
	
	default double supplyCurrent() {
		return statorCurrent() * outputVoltage() / 12.0;
	}
}
