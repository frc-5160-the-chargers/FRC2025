package frc.chargers.hardware.motorcontrol;

import frc.chargers.utils.PIDConstants;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.chargers.hardware.encoders.Encoder;
import lombok.With;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

@Logged
public interface Motor extends AutoCloseable {
	@With
	record CommonConfig(
		double gearRatio,
		PIDConstants positionPID,
		PIDConstants velocityPID,
		boolean continuousInput
	){
		public static final CommonConfig EMPTY =
			new CommonConfig(1.0, PIDConstants.VOID, PIDConstants.VOID, false);
	}
	
	Encoder encoder();
	double outputVoltage();
	double statorCurrent();
	double tempCelsius();
	
	void setCommonConfig(CommonConfig newConfig);
	void setVoltage(double volts);
	void setVelocity(double velocityRadPerSec, double ffVolts);
	void moveToPosition(double positionRads, double ffVolts);
	void setTorqueCurrent(double currentAmps);
	
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
