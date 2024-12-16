package frc.chargers.hardware.motorcontrol;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.units.measure.*;
import frc.chargers.hardware.encoders.Encoder;
import lombok.experimental.FieldDefaults;

import static edu.wpi.first.math.util.Units.rotationsToRadians;

@FieldDefaults(makeFinal = true)
public class ChargerTalonFX implements Motor, AutoCloseable {
	protected TalonFX baseMotor;
	private StatusSignal<Angle> positionSignal;
	private StatusSignal<AngularVelocity> velocitySignal;
	private StatusSignal<Voltage> voltageSignal;
	private StatusSignal<Current> currentSignal;
	private StatusSignal<Current> supplyCurrentSignal;
	private StatusSignal<Temperature> tempSignal;
	
	private VoltageOut voltageRequest = new VoltageOut(0.0);
	private PositionVoltage setAngleRequest = new PositionVoltage(0.0);
	private VelocityVoltage setVelocityRequest = new VelocityVoltage(0.0);
	private TorqueCurrentFOC setCurrentRequest = new TorqueCurrentFOC(0.0);
	
	private Encoder encoderIO = new Encoder() {
		@Override
		public double positionRad() {
			return rotationsToRadians(positionSignal.refresh().getValueAsDouble());
		}
		
		@Override
		public double velocityRadPerSec() {
			return rotationsToRadians(velocitySignal.refresh().getValueAsDouble());
		}
		
		@Override
		public void setPositionReading(Angle angle) {
			baseMotor.setPosition(angle);
		}
	};
	
	public ChargerTalonFX(int id, double gearRatio) {
		this.baseMotor = new TalonFX(id);
		this.positionSignal = baseMotor.getPosition();
		this.velocitySignal = baseMotor.getVelocity();
		this.voltageSignal = baseMotor.getMotorVoltage();
		this.currentSignal = baseMotor.getStatorCurrent();
		this.supplyCurrentSignal = baseMotor.getSupplyCurrent();
		this.tempSignal = baseMotor.getDeviceTemp();
		baseMotor.getConfigurator().apply(
			new FeedbackConfigs().withSensorToMechanismRatio(gearRatio)
		);
	}
	
	public ChargerTalonFX withPhoenixPro() {
		voltageRequest.EnableFOC = true;
		setAngleRequest.EnableFOC = true;
		setVelocityRequest.EnableFOC = true;
		return this;
	}
	
	public TalonFXConfigurator getConfigurator() {
		return baseMotor.getConfigurator();
	}
	
	public ChargerTalonFX configure(TalonFXConfiguration config) {
		baseMotor.getConfigurator().apply(config);
		return this;
	}
	
	@Override
	public Encoder encoder() { return encoderIO; }
	
	@Override
	public double outputVoltage() { return voltageSignal.refresh().getValueAsDouble(); }
	
	@Override
	public double statorCurrent() { return currentSignal.refresh().getValueAsDouble(); }
	
	@Override
	public double tempCelsius() { return tempSignal.refresh().getValueAsDouble(); }
	
	@Override
	public void setVoltage(double volts) {
		voltageRequest.Output = volts;
		baseMotor.setControl(voltageRequest);
	}
	
	@Override
	public void setVelocity(double velocityRadPerSec, double ffVolts) {
		setVelocityRequest.Velocity = rotationsToRadians(velocityRadPerSec);
		setVelocityRequest.FeedForward = ffVolts;
		baseMotor.setControl(setVelocityRequest);
	}
	
	@Override
	public void moveToPosition(double positionRads, double ffVolts) {
		setAngleRequest.Position = rotationsToRadians(positionRads);
		setAngleRequest.FeedForward = ffVolts;
		baseMotor.setControl(setAngleRequest);
	}
	
	@Override
	public void setTorqueCurrent(double currentAmps) {
		setCurrentRequest.Output = currentAmps;
		baseMotor.setControl(setCurrentRequest);
	}
	
	@Override
	public void setCoastMode(boolean on) {
		var config = new MotorOutputConfigs();
		getConfigurator().refresh(config);
		config.NeutralMode = on ? NeutralModeValue.Coast : NeutralModeValue.Brake;
		getConfigurator().apply(config);
	}
	
	@Override
	public void setPositionPID(PIDConstants constants) {
		baseMotor.getConfigurator().apply(
			new Slot0Configs()
				.withKP(constants.kP)
				.withKI(constants.kI)
				.withKD(constants.kD)
		);
	}
	
	@Override
	public void setVelocityPID(PIDConstants constants) {
		baseMotor.getConfigurator().apply(
			new Slot1Configs()
				.withKP(constants.kP)
				.withKI(constants.kI)
				.withKD(constants.kD)
		);
	}
	
	@Override
	public void enableContinuousInput() {
		var config = new ClosedLoopGeneralConfigs();
		config.ContinuousWrap = true;
		baseMotor.getConfigurator().apply(config);
	}
	
	@Override
	public double supplyCurrent() {
		return supplyCurrentSignal.refresh().getValueAsDouble();
	}
	
	@Override
	public void close(){ baseMotor.close(); }
}
