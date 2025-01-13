package frc.chargers.hardware.encoders;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

@Logged
public interface Encoder extends AutoCloseable {
	double positionRad();
	double velocityRadPerSec();
	void setPositionReading(Angle angle);
	
	@NotLogged default Angle position() { return Radians.of(positionRad()); }
	@NotLogged default AngularVelocity velocity() { return RadiansPerSecond.of(velocityRadPerSec()); }
	default void close() {}
}
