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
	public final CANcoder baseApi;
	protected final StatusSignal<Angle> positionSignal;
	protected final StatusSignal<AngularVelocity> velocitySignal;
	
	public ChargerCANcoder(int id, boolean optimizeBusUtilization, @Nullable CANcoderConfiguration config) {
		this.baseApi = new CANcoder(id);
		this.positionSignal = baseApi.getAbsolutePosition();
		this.velocitySignal = baseApi.getVelocity();
		StatusSignalRefresher.addSignals(positionSignal, velocitySignal);
		BaseStatusSignal.setUpdateFrequencyForAll(50, positionSignal, velocitySignal);
		if (optimizeBusUtilization) baseApi.optimizeBusUtilization();
		if (config != null) tryUntilOk(baseApi, () -> baseApi.getConfigurator().apply(config, 0.1));
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
		tryUntilOk(baseApi, () -> baseApi.setPosition(angle));
	}
	
	@Override
	public void close() {
		if (RobotBase.isSimulation()) baseApi.close();
	}
}
