package frc.chargers.hardware.motorcontrol;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.chargers.hardware.encoders.Encoder;
import frc.chargers.utils.StatusSignalRefresher;
import org.jetbrains.annotations.Nullable;

import java.util.HashSet;
import java.util.Set;

import static edu.wpi.first.math.util.Units.radiansToRotations;
import static edu.wpi.first.math.util.Units.rotationsToRadians;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.chargers.utils.UtilMethods.tryUntilOk;
import static java.lang.Math.PI;

/**
 * A TalonFX that implements the Motor interface.
 * To access more low-level capabilities of the base api, inherit from this class.
 */
public class ChargerTalonFX implements Motor {
	private static final Set<Integer> usedIds = new HashSet<>();
	public static int getDummyId() {
		int id = 0;
		while (usedIds.contains(id)) {
			id = (int) (Math.random() * 31);
		}
		return id;
	}
	
	protected final TalonFX baseApi;
	private double currentGearRatio = 1.0;
	private boolean useTorqueCurrentControl = false;
	private boolean highFrequencyPosition = false;
	private final StatusSignal<?> positionSignal, velocitySignal, voltageSignal,
		currentSignal, torqueCurrentSignal, supplyCurrentSignal, tempSignal;
	
	protected final VoltageOut setVoltageRequest = new VoltageOut(0);
	protected final PositionVoltage setAngleRequest = new PositionVoltage(0).withSlot(0);
	protected final VelocityVoltage setVelocityRequest = new VelocityVoltage(0).withSlot(1);
	protected final PositionTorqueCurrentFOC setAngleWithTorqueRequest = new PositionTorqueCurrentFOC(0).withSlot(0);
	protected final VelocityTorqueCurrentFOC setVelocityWithTorqueRequest = new VelocityTorqueCurrentFOC(0).withSlot(1);
	protected final TorqueCurrentFOC setTorqueRequest = new TorqueCurrentFOC(0);
	
	private final Encoder encoder = new Encoder() {
		@Override
		public double positionRad() {
			if (highFrequencyPosition) positionSignal.refresh();
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
	};
	
	public ChargerTalonFX(int id, boolean optimizeBusUtilization, @Nullable TalonFXConfiguration config) {
		this.baseApi = new TalonFX(id);
		usedIds.add(id);
		this.positionSignal = baseApi.getPosition();
		this.velocitySignal = baseApi.getVelocity();
		this.voltageSignal = baseApi.getMotorVoltage();
		this.currentSignal = baseApi.getStatorCurrent();
		this.supplyCurrentSignal = baseApi.getSupplyCurrent();
		this.tempSignal = baseApi.getDeviceTemp();
		this.torqueCurrentSignal = baseApi.getTorqueCurrent();
		StatusSignal<?>[] allSignals = {
			positionSignal, velocitySignal, voltageSignal, currentSignal,
			supplyCurrentSignal, tempSignal, torqueCurrentSignal
		};
		StatusSignalRefresher.addSignals(allSignals);
		BaseStatusSignal.setUpdateFrequencyForAll(50, allSignals);
		if (optimizeBusUtilization) baseApi.optimizeBusUtilization();
		if (config != null) tryUntilOk(baseApi, () -> baseApi.getConfigurator().apply(config, 0.1));
	}
	
	public ChargerTalonFX withSim(SimDynamics dynamics) {
		if (!RobotBase.isSimulation()) return this;
		var dataInjector = baseApi.getSimState();
		HAL.registerSimPeriodicAfterCallback(() -> {
			dynamics.acceptVolts().accept(MathUtil.clamp(dataInjector.getMotorVoltage(), -12, 12));
			dataInjector.setSupplyVoltage(RobotController.getBatteryVoltage());
			dataInjector.setRawRotorPosition(Radians.of(dynamics.position().getAsDouble() * currentGearRatio));
			dataInjector.setRotorVelocity(RadiansPerSecond.of(dynamics.velocity().getAsDouble() * currentGearRatio));
		});
		return this;
	}
	
	public ChargerTalonFX enablePhoenixPro(boolean useTorqueCurrentControl) {
		setVoltageRequest.EnableFOC = true;
		setAngleRequest.EnableFOC = true;
		setVelocityRequest.EnableFOC = true;
		this.useTorqueCurrentControl = useTorqueCurrentControl;
		return this;
	}
	
	public ChargerTalonFX setPositionUpdateRate(double frequencyHz) {
		positionSignal.setUpdateFrequency(frequencyHz);
		StatusSignalRefresher.remove(positionSignal);
		highFrequencyPosition = true;
		return this;
	}
	
	@Override
	public Encoder encoder() { return encoder; }
	
	@Override
	public double outputVoltage() { return voltageSignal.getValueAsDouble(); }
	
	@Override
	public double statorCurrent() { return currentSignal.getValueAsDouble(); }
	
	@Override
	public double tempCelsius() { return tempSignal.getValueAsDouble(); }
	
	@Override
	public void setVoltage(double volts) {
		baseApi.setControl(setVoltageRequest.withOutput(volts));
	}
	
	@Override
	public void setVelocity(double velocityRadPerSec, double ffVolts) {
		if (useTorqueCurrentControl) {
			setVelocityWithTorqueRequest.Velocity = radiansToRotations(velocityRadPerSec);
			setVelocityWithTorqueRequest.FeedForward = ffVolts;
			baseApi.setControl(setVelocityWithTorqueRequest);
		} else {
			setVelocityRequest.Velocity = radiansToRotations(velocityRadPerSec);
			setVelocityRequest.FeedForward = ffVolts;
			baseApi.setControl(setVelocityRequest);
		}
	}
	
	@Override
	public int id() {
		return baseApi.getDeviceID();
	}
	
	@Override
	public void moveToPosition(double positionRads, double ffVolts) {
		if (useTorqueCurrentControl) {
			setAngleWithTorqueRequest.Position = radiansToRotations(positionRads);
			setAngleWithTorqueRequest.FeedForward = ffVolts;
			baseApi.setControl(setAngleWithTorqueRequest);
		} else {
			setAngleRequest.Position = radiansToRotations(positionRads);
			setAngleRequest.FeedForward = ffVolts;
			baseApi.setControl(setAngleRequest);
		}
	}
	
	@Override
	public void setCoastMode(boolean enabled) {
		baseApi.setNeutralMode(enabled ? NeutralModeValue.Coast : NeutralModeValue.Brake);
	}
	
	@Override
	public void setControlsConfig(ControlsConfig newConfig) {
		var motorConfig = new TalonFXConfiguration();
		tryUntilOk(baseApi, () -> baseApi.getConfigurator().refresh(motorConfig, 0.1));
		if (newConfig.gearRatio() != 1.0) {
			if (motorConfig.Feedback.FeedbackSensorSource == FeedbackSensorSourceValue.RotorSensor) {
				motorConfig.Feedback.SensorToMechanismRatio = newConfig.gearRatio();
			} else {
				motorConfig.Feedback.RotorToSensorRatio = newConfig.gearRatio();
			}
			currentGearRatio = newConfig.gearRatio();
		}
		if (newConfig.positionPID().kP != 0.0) {
			motorConfig.Slot0.kP = newConfig.positionPID().kP * (2 * PI);
			motorConfig.Slot0.kI = newConfig.positionPID().kI * (2 * PI);
			motorConfig.Slot0.kD = newConfig.positionPID().kD * (2 * PI);
			motorConfig.ClosedLoopGeneral.ContinuousWrap = newConfig.continuousInput();
		}
		if (newConfig.velocityPID().kP != 0.0) {
			motorConfig.Slot1.kP = newConfig.velocityPID().kP * (2 * PI);
			motorConfig.Slot1.kI = newConfig.velocityPID().kI * (2 * PI);
			motorConfig.Slot1.kD = newConfig.velocityPID().kD * (2 * PI);
		}
		tryUntilOk(baseApi, () -> baseApi.getConfigurator().apply(motorConfig, 0.1));
	}
	
	@Override
	public double supplyCurrent() { return supplyCurrentSignal.getValueAsDouble(); }
	
	@Override
	public double torqueCurrent() { return torqueCurrentSignal.getValueAsDouble(); }
	
	@Override
	public void setTorqueCurrent(double currentAmps) {
		baseApi.setControl(setTorqueRequest.withOutput(currentAmps));
	}
	
	@Override
	public void close() {
		if (RobotBase.isSimulation()) baseApi.close();
	}
}
