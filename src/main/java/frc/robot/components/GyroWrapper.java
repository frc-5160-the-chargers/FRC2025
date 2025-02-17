package frc.robot.components;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.chargers.utils.StatusSignalRefresher;

@Logged
public class GyroWrapper {
	@NotLogged private final Pigeon2 pigeon = new Pigeon2(0);
	private final BaseStatusSignal
		yaw = pigeon.getYaw(),
		pitch = pigeon.getPitch(),
		roll = pigeon.getRoll();
	
	public final Trigger isTipping = new Trigger(() -> false);
	
	public GyroWrapper() {
		// automatically calls refresh() on signals
		StatusSignalRefresher.addSignals(yaw, pitch, roll);
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
}
