package frc.chargers.hardware.encoders;


import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

import static edu.wpi.first.math.util.Units.rotationsToRadians;

@RequiredArgsConstructor
public class CANcoderIO implements EncoderIO, AutoCloseable {
	@Getter private final CANcoder baseEncoder;
	private final StatusSignal<Angle> positionSignal = baseEncoder.getPosition();
	private final StatusSignal<AngularVelocity> velocitySignal = baseEncoder.getVelocity();
	
	@Override
	public double positionRad() {
		return rotationsToRadians(positionSignal.refresh().getValueAsDouble());
	}
	
	@Override
	public double velocityRadPerSec() {
		return rotationsToRadians(velocitySignal.refresh().getValueAsDouble());
	}
	
	@Override
	public void close() { baseEncoder.close(); }
}
