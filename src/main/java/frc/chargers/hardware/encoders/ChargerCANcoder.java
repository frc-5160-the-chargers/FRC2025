package frc.chargers.hardware.encoders;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import frc.chargers.utils.StatusSignalRefresher;
import org.jetbrains.annotations.Nullable;

import static edu.wpi.first.math.util.Units.rotationsToRadians;
import static frc.chargers.utils.UtilMethods.tryUntilOk;

public class ChargerCANcoder implements Encoder {
	protected final CANcoder baseEncoder;
	protected final StatusSignal<Angle> positionSignal;
	protected final StatusSignal<AngularVelocity> velocitySignal;
	
	public ChargerCANcoder(int id, boolean optimizeBusUtilization, @Nullable CANcoderConfiguration config) {
		this.baseEncoder = new CANcoder(id);
		this.positionSignal = baseEncoder.getAbsolutePosition();
		this.velocitySignal = baseEncoder.getVelocity();
		StatusSignalRefresher.addSignals(positionSignal, velocitySignal);
		BaseStatusSignal.setUpdateFrequencyForAll(50, positionSignal, velocitySignal);
		if (optimizeBusUtilization) baseEncoder.optimizeBusUtilization();
		if (config != null) tryUntilOk(baseEncoder, () -> baseEncoder.getConfigurator().apply(config, 0.01));
	}
	
	@Override
	public double positionRad() {
		return rotationsToRadians(positionSignal.getValueAsDouble());
	}
	
	@Override
	public double velocityRadPerSec() {
		return rotationsToRadians(velocitySignal.getValueAsDouble());
	}
	
	@Override
	public void setPositionReading(Angle angle) {
		baseEncoder.setPosition(angle);
	}
	
	@Override
	public void close() {
		if (RobotBase.isSimulation()) baseEncoder.close();
	}
}
