package frc.chargers.hardware.encoders;


import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.math.util.Units.rotationsToRadians;

public class CANcoderIO implements EncoderIO, AutoCloseable {
	private final CANcoder baseEncoder;
	private final StatusSignal<Angle> positionSignal;
	private final StatusSignal<AngularVelocity> velocitySignal;
	
	public CANcoderIO(CANcoder baseEncoder) {
		this.baseEncoder = baseEncoder;
		this.positionSignal = baseEncoder.getPosition();
		this.velocitySignal = baseEncoder.getVelocity();
	}
	
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
