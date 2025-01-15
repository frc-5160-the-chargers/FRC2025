package frc.chargers.hardware.motorcontrol;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import frc.chargers.hardware.encoders.Encoder;
import frc.chargers.utils.SignalAutoRefresher;
import org.jetbrains.annotations.Nullable;

import java.util.function.Consumer;

import static edu.wpi.first.math.util.Units.radiansToRotations;
import static edu.wpi.first.math.util.Units.rotationsToRadians;
import static java.lang.Math.PI;

public class ChargerTalonFX implements Motor {
	// package-private for unit tests
	final TalonFX baseMotor;
	private boolean hasFusedSensor = false;
	private boolean highFrequencyPosition = false;
	private boolean useTorqueCurrentControl = false;
	private final StatusSignal<Angle> positionSignal;
	private final StatusSignal<AngularVelocity> velocitySignal;
	private final StatusSignal<Voltage> voltageSignal;
	private final StatusSignal<Current> currentSignal;
	private final StatusSignal<Current> torqueCurrentSignal;
	private final StatusSignal<Current> supplyCurrentSignal;
	private final StatusSignal<Temperature> tempSignal;
	
	private final VoltageOut voltageRequest = new VoltageOut(0);
	private final PositionVoltage setAngleRequest = new PositionVoltage(0).withSlot(0);
	private final VelocityVoltage setVelocityRequest = new VelocityVoltage(0).withSlot(1);
	private final PositionTorqueCurrentFOC setAngleWithTorqueRequest = new PositionTorqueCurrentFOC(0).withSlot(0);
	private final VelocityTorqueCurrentFOC setVelocityWithTorqueRequest = new VelocityTorqueCurrentFOC(0).withSlot(1);
	private final TorqueCurrentFOC setTorqueRequest = new TorqueCurrentFOC(0);
	
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
			baseMotor.setPosition(angle);
		}
	};
	
	public ChargerTalonFX(int id, @Nullable Consumer<TalonFXConfigurator> configureFn) {
		this(id, configureFn, true);
	}
	
	public ChargerTalonFX(int id, @Nullable Consumer<TalonFXConfigurator> configureFn, boolean disableUnusedSignals) {
		this.baseMotor = new TalonFX(id);
		this.positionSignal = baseMotor.getPosition();
		this.velocitySignal = baseMotor.getVelocity();
		this.voltageSignal = baseMotor.getMotorVoltage();
		this.currentSignal = baseMotor.getStatorCurrent();
		this.supplyCurrentSignal = baseMotor.getSupplyCurrent();
		this.tempSignal = baseMotor.getDeviceTemp();
		this.torqueCurrentSignal = baseMotor.getTorqueCurrent();
		StatusSignal<?>[] allSignals = {
			positionSignal, velocitySignal, voltageSignal, currentSignal,
			supplyCurrentSignal, tempSignal, torqueCurrentSignal
		};
		SignalAutoRefresher.register(allSignals);
		if (disableUnusedSignals) {
			BaseStatusSignal.setUpdateFrequencyForAll(50.0, allSignals);
			baseMotor.optimizeBusUtilization();
		}
		if (configureFn != null) configureFn.accept(baseMotor.getConfigurator());
	}
	
	public ChargerTalonFX withPhoenixPro(boolean useTorqueCurrentControl) {
		voltageRequest.EnableFOC = true;
		setAngleRequest.EnableFOC = true;
		setVelocityRequest.EnableFOC = true;
		this.useTorqueCurrentControl = useTorqueCurrentControl;
		return this;
	}
	
	public ChargerTalonFX withPositionUpdateFrequency(Frequency frequency) {
		positionSignal.setUpdateFrequency(frequency);
		SignalAutoRefresher.remove(positionSignal);
		highFrequencyPosition = true;
		return this;
	}
	
	public ChargerTalonFX withFusedSensor(CANcoder canCoder) {
		hasFusedSensor = true;
		var config = new FeedbackConfigs();
		baseMotor.getConfigurator().refresh(config);
		config.FeedbackRemoteSensorID = canCoder.getDeviceID();
		config.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
		baseMotor.getConfigurator().apply(config);
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
		baseMotor.setControl(voltageRequest.withOutput(volts));
	}
	
	@Override
	public void setVelocity(double velocityRadPerSec, double ffVolts) {
		if (useTorqueCurrentControl) {
			setVelocityWithTorqueRequest.Velocity = radiansToRotations(velocityRadPerSec);
			setVelocityWithTorqueRequest.FeedForward = ffVolts;
			baseMotor.setControl(setVelocityWithTorqueRequest);
		} else {
			setVelocityRequest.Velocity = radiansToRotations(velocityRadPerSec);
			setVelocityRequest.FeedForward = ffVolts;
			baseMotor.setControl(setVelocityRequest);
		}
	}
	
	@Override
	public void moveToPosition(double positionRads, double ffVolts) {
		if (useTorqueCurrentControl) {
			setAngleWithTorqueRequest.Position = positionRads;
			setAngleWithTorqueRequest.FeedForward = ffVolts;
			baseMotor.setControl(setAngleWithTorqueRequest);
		} else {
			setAngleRequest.Position = radiansToRotations(positionRads);
			setAngleRequest.FeedForward = ffVolts;
			baseMotor.setControl(setAngleRequest);
		}
	}
	
	@Override
	public void setControlsConfig(ControlsConfig newConfig) {
		var motorConfig = new TalonFXConfiguration();
		baseMotor.getConfigurator().refresh(motorConfig);
		if (newConfig.gearRatio() != 1.0) {
			if (hasFusedSensor) {
				motorConfig.Feedback.RotorToSensorRatio = newConfig.gearRatio();
			} else {
				motorConfig.Feedback.SensorToMechanismRatio = newConfig.gearRatio();
			}
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
		baseMotor.getConfigurator().apply(motorConfig, 0.01);
	}
	
	@Override
	public double supplyCurrent() { return supplyCurrentSignal.getValueAsDouble(); }
	
	@Override
	public double torqueCurrent() { return torqueCurrentSignal.getValueAsDouble(); }
	
	@Override
	public void setTorqueCurrent(double currentAmps) {
		baseMotor.setControl(setTorqueRequest.withOutput(currentAmps));
	}
	
	@Override
	public void close() {
		if (RobotBase.isReal()) return;
		baseMotor.close();
	}
}
