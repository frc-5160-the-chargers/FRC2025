package frc.chargers.hardware.motorcontrol;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.chargers.hardware.encoders.EncoderIO;
import lombok.Getter;
import lombok.experimental.FieldDefaults;

import static com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters;
import static com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters;
import static com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters;
import static com.revrobotics.spark.config.ClosedLoopConfig.ClosedLoopSlot.kSlot0;
import static com.revrobotics.spark.config.ClosedLoopConfig.ClosedLoopSlot.kSlot1;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast;
import static edu.wpi.first.math.util.Units.rotationsToRadians;
import static edu.wpi.first.units.Units.Rotations;

@FieldDefaults(makeFinal = true)
public class SparkIO<BaseMotor extends SparkBase> implements MotorIO, AutoCloseable {
	@Getter private BaseMotor baseMotor;
	private RelativeEncoder baseEncoder;
	private SparkClosedLoopController pidController;
	private EncoderIO encoderIO = new EncoderIO() {
		@Override
		public double positionRad() {
			return rotationsToRadians(baseEncoder.getPosition());
		}
		
		@Override
		public double velocityRadPerSec() {
			return rotationsToRadians(baseEncoder.getVelocity());
		}
		
		@Override
		public void setPositionReading(Angle angle) {
			baseEncoder.setPosition(angle.in(Rotations));
		}
	};
	
	public SparkIO(BaseMotor baseMotor, double gearRatio) {
		this.baseMotor = baseMotor;
		this.baseEncoder = baseMotor.getEncoder();
		this.pidController = baseMotor.getClosedLoopController();
		baseMotor.configure(
			new SparkMaxConfig().apply(
				new EncoderConfig()
					.positionConversionFactor(1 / gearRatio)
					.velocityConversionFactor(1 / gearRatio)
			),
			kResetSafeParameters, kPersistParameters
		);
	}
	
	@Override
	public EncoderIO encoder() { return encoderIO; }
	
	@Override
	public double outputVoltage() {
		return baseMotor.getAppliedOutput() * baseMotor.getBusVoltage();
	}
	
	@Override
	public double statorCurrent() { return baseMotor.getOutputCurrent(); }
	
	@Override
	public double tempCelsius() { return baseMotor.getMotorTemperature(); }
	
	@Override
	public void setVoltage(double volts) { baseMotor.setVoltage(volts); }
	
	@Override
	public void setVelocity(double velocityRadPerSec, double ffVolts) {
		pidController.setReference(
			Units.radiansToRotations(velocityRadPerSec),
			SparkBase.ControlType.kVelocity,
			1
		);
	}
	
	@Override
	public void setCoastMode(boolean on) {
		baseMotor.configure(
			new SparkMaxConfig().idleMode(on ? kBrake : kCoast),
			kNoResetSafeParameters, kPersistParameters
		);
	}
	
	@Override
	public void setTorqueCurrent(double currentAmps) {}
	
	@Override
	public void moveToPosition(double positionRads, double ffVolts) {
		pidController.setReference(
			Units.radiansToRotations(positionRads),
			SparkBase.ControlType.kVelocity,
			2
		);
	}
	
	@Override
	public void setPositionPID(double p, double i, double d) {
		var config = new SparkMaxConfig();
		config.closedLoop.p(p, kSlot0).i(i, kSlot0).d(d, kSlot0);
		baseMotor.configure(config, kNoResetSafeParameters, kPersistParameters);
	}
	
	@Override
	public void setVelocityPID(double p, double i, double d) {
		var config = new SparkMaxConfig();
		config.closedLoop.p(p, kSlot1).i(i, kSlot1).d(d, kSlot1);
		baseMotor.configure(config, kNoResetSafeParameters, kPersistParameters);
	}
	
	@Override
	public void enableContinuousInput() {
		// TODO
	}
	
	@Override
	public void close(){ baseMotor.close(); }
}
