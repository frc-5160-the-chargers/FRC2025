package frc.chargers.hardware.encoders;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

@Logged
public interface EncoderIO {
	static CANcoderIO of(CANcoder encoder) { return new CANcoderIO(encoder); }
	static EncoderIO ofDutyCycle(int port, boolean inverted) {
		var encoder = new DutyCycleEncoder(port);
		return new EncoderIO() {
			@Override
			public double positionRad() { return encoder.get() * (inverted ? -1 : 1); }
			
			@Override
			public double velocityRadPerSec() { return 0; }
		};
	}
	
	double positionRad();
	double velocityRadPerSec();
	
	@NotLogged default Angle position() { return Radians.of(positionRad()); }
	@NotLogged default AngularVelocity velocity() { return RadiansPerSecond.of(velocityRadPerSec()); }
}
