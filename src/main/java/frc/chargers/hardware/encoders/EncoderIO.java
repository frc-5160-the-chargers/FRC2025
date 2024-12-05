package frc.chargers.hardware.encoders;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import static edu.wpi.first.math.util.Units.rotationsToRadians;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

@Logged
public interface EncoderIO {
	static CANcoderIO of(CANcoder encoder) { return new CANcoderIO(encoder); }
	static EncoderIO ofDutyCycle(int port, boolean inverted) {
		return new EncoderIO() {
			private final DutyCycleEncoder encoder = new DutyCycleEncoder(port);
			private double offset = 0.0;
			
			@Override
			public double positionRad() {
				return rotationsToRadians(encoder.get() - offset) * (inverted ? -1 : 1);
			}
			
			@Override
			public double velocityRadPerSec() { return 0; }
			
			@Override
			public void setPositionReading(Angle angle) { offset = encoder.get();  }
		};
	}
	
	double positionRad();
	double velocityRadPerSec();
	void setPositionReading(Angle angle);
	
	@NotLogged default Angle position() { return Radians.of(positionRad()); }
	@NotLogged default AngularVelocity velocity() { return RadiansPerSecond.of(velocityRadPerSec()); }
}
