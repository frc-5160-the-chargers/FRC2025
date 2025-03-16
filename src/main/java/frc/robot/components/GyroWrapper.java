package frc.robot.components;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.chargers.utils.data.StatusSignalRefresher;
import frc.robot.subsystems.swerve.SwerveConfigurator;

@Logged
public class GyroWrapper {
	@NotLogged private final Pigeon2 pigeon = new Pigeon2(0);
	private final BaseStatusSignal
		yaw = pigeon.getYaw(),
		pitch = pigeon.getPitch(),
		roll = pigeon.getRoll(),
		pitchRate = pigeon.getAngularVelocityYWorld(),
		rollRate = pigeon.getAngularVelocityXWorld();
	
	public final Trigger isTipping = new Trigger(
		() -> Math.abs(pitch().getDegrees()) > 25 || Math.abs(roll().getDegrees()) > 25
	);
	
	public GyroWrapper() {
		yaw.setUpdateFrequency(SwerveConfigurator.ODOMETRY_FREQUENCY_HZ);
		// automatically calls refresh() on signals
		StatusSignalRefresher.addSignals(pitch, roll, pitchRate, rollRate);
	}
	
	public Rotation2d yaw() {
		BaseStatusSignal.refreshAll(yaw);
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
