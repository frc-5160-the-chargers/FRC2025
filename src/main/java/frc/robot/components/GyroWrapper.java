package frc.robot.components;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.chargers.utils.data.StatusSignalRefresher;
import frc.robot.subsystems.swerve.SwerveConfigurator;
import lombok.Getter;
import monologue.LogLocal;

@Logged
public class GyroWrapper implements LogLocal {
	@NotLogged private final Pigeon2 pigeon = new Pigeon2(0);
	@NotLogged private final BaseStatusSignal
		yaw = pigeon.getYaw(),
		pitch = pigeon.getPitch(),
		roll = pigeon.getRoll(),
		pitchRate = pigeon.getAngularVelocityYWorld(),
		rollRate = pigeon.getAngularVelocityXWorld();
	@Getter private double lastLatency = 0.0;
	
	public final Trigger isTipping = new Trigger(
		() -> Math.abs(pitch().getDegrees()) > 25 || Math.abs(roll().getDegrees()) > 25
	);
	
	public GyroWrapper() {
		yaw.setUpdateFrequency(SwerveConfigurator.ODOMETRY_FREQUENCY_HZ);
		// automatically calls refresh() on signals
		StatusSignalRefresher.addSignals(pitch, roll, pitchRate, rollRate);
	}
	
	/** Refreshes the yaw signal of the gyro wrapper. Must be placed in an addPeriodic. */
	public void refreshYaw() {
		lastLatency = yaw.getTimestamp().getLatency();
		BaseStatusSignal.refreshAll(yaw);
	}
	
	public Rotation2d yaw() {
		return Rotation2d.fromDegrees(yaw.getValueAsDouble());
	}
	
	public Rotation2d pitch() {
		return Rotation2d.fromDegrees(pitch.getValueAsDouble());
	}
	
	public Rotation2d roll() {
		return Rotation2d.fromDegrees(roll.getValueAsDouble());
	}
	
	public double pitchRateDegPerSec() {
		return pitchRate.getValueAsDouble();
	}
	
	public double rollRateDegPerSec() {
		return rollRate.getValueAsDouble();
	}
}
