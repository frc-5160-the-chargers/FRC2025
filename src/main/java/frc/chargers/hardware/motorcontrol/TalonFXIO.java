package frc.chargers.hardware.motorcontrol;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.chargers.hardware.encoders.EncoderIO;
import frc.chargers.utils.ChargerExtensions;
import lombok.Getter;
import lombok.experimental.ExtensionMethod;
import lombok.experimental.FieldDefaults;

import static edu.wpi.first.math.util.Units.rotationsToRadians;

@FieldDefaults(makeFinal = true)
@ExtensionMethod(ChargerExtensions.class)
public class TalonFXIO implements MotorIO, AutoCloseable {
	@Getter private TalonFX baseMotor;
	private StatusSignal<Angle> positionSignal;
	private StatusSignal<AngularVelocity> velocitySignal;
	private StatusSignal<Voltage> voltageSignal;
	private StatusSignal<Current> currentSignal;
	private StatusSignal<Temperature> tempSignal;
	
	private VoltageOut voltageRequest = new VoltageOut(0.0);
	private PositionVoltage setAngleRequest = new PositionVoltage(0.0);
	private VelocityVoltage setVelocityRequest = new VelocityVoltage(0.0);
	private TorqueCurrentFOC setCurrentRequest = new TorqueCurrentFOC(0.0);
	
	private EncoderIO encoderIO = new EncoderIO() {
		@Override
		public double positionRad() {
			return rotationsToRadians(positionSignal.refresh().getValueAsDouble());
		}
		
		@Override
		public double velocityRadPerSec() {
			return rotationsToRadians(velocitySignal.refresh().getValueAsDouble());
		}
	};
	
	public TalonFXIO(TalonFX baseMotor, double gearRatio) {
		var feedbackConfigs = new FeedbackConfigs();
		baseMotor.getConfigurator().refresh(feedbackConfigs);
		baseMotor.getConfigurator().apply(feedbackConfigs.withSensorToMechanismRatio(gearRatio));
		
		this.baseMotor = baseMotor;
		this.positionSignal = baseMotor.getPosition();
		this.velocitySignal = baseMotor.getVelocity();
		this.voltageSignal = baseMotor.getMotorVoltage();
		this.currentSignal = baseMotor.getSupplyCurrent();
		this.tempSignal = baseMotor.getDeviceTemp();
	}
	
	public TalonFXIO withPhoenixPro() {
		voltageRequest.EnableFOC = true;
		setAngleRequest.EnableFOC = true;
		setVelocityRequest.EnableFOC = true;
		return this;
	}
	
	@Override
	public EncoderIO encoder() { return encoderIO; }
	
	@Override
	public double outputVoltageVolts() { return voltageSignal.refresh().getValueAsDouble(); }
	
	@Override
	public double currentDrawAmps() { return currentSignal.refresh().getValueAsDouble(); }
	
	@Override
	public double tempCelsius() { return tempSignal.refresh().getValueAsDouble(); }
	
	@Override
	public void setVoltage(double volts) {
		voltageRequest.Output = volts;
		baseMotor.setControl(voltageRequest);
	}
	
	@Override
	public void spinAtVelocity(double velocityRadPerSec, double ffVolts) {
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
		baseMotor.getConfigurator().apply(
			new MotorOutputConfigs()
				.also(it -> baseMotor.getConfigurator().refresh(it))
				.withNeutralMode(on ? NeutralModeValue.Coast : NeutralModeValue.Brake)
		);
	}
	
	@Override
	public void setPositionPID(double p, double i, double d) {
		baseMotor.getConfigurator().apply(
			new Slot0Configs()
				.also(it -> baseMotor.getConfigurator().refresh(it))
				.withKP(p)
				.withKI(i)
				.withKD(d)
		);
	}
	
	@Override
	public void setVelocityPID(double p, double i, double d) {
		baseMotor.getConfigurator().apply(
			new Slot1Configs()
				.also(it -> baseMotor.getConfigurator().refresh(it))
				.withKP(p)
				.withKI(i)
				.withKD(d)
		);
	}
	
	@Override
	public void close(){ baseMotor.close(); }
}
