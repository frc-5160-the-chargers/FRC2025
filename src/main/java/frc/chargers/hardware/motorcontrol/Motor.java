package frc.chargers.hardware.motorcontrol;

import edu.wpi.first.wpilibj.Alert;
import frc.chargers.utils.PIDConstants;
import edu.wpi.first.epilogue.Logged;
import frc.chargers.hardware.encoders.Encoder;
import lombok.With;

import static edu.wpi.first.wpilibj.Alert.AlertType.kError;

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
	Alert torqueCtrlNotAvailable = new Alert("SetTorqueCurrent not implement on Motor", kError);
	
	Encoder encoder();
	double outputVoltage();
	double statorCurrent();
	double tempCelsius();
	
	void setCommonConfig(CommonConfig newConfig);
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
	default void setTorqueCurrent(double currentAmps) { torqueCtrlNotAvailable.set(true); }
	default void close() {}
}
