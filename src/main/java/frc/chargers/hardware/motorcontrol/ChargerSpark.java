package frc.chargers.hardware.motorcontrol;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.chargers.hardware.encoders.Encoder;
import lombok.Getter;
import lombok.experimental.FieldDefaults;

import static com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters;
import static com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters;
import static com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters;
import static com.revrobotics.spark.SparkLowLevel.*;
import static com.revrobotics.spark.config.ClosedLoopConfig.ClosedLoopSlot.kSlot0;
import static com.revrobotics.spark.config.ClosedLoopConfig.ClosedLoopSlot.kSlot1;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast;
import static edu.wpi.first.math.util.Units.rotationsToRadians;
import static edu.wpi.first.units.Units.Rotations;
import static java.lang.Math.PI;

@FieldDefaults(makeFinal = true)
public class ChargerSpark<BaseMotor extends SparkBase> implements Motor, AutoCloseable {
	@Getter private BaseMotor baseMotor;
	private RelativeEncoder baseEncoder;
	private SparkClosedLoopController pidController;
	private Encoder encoder = new Encoder() {
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
	
	public static ChargerSpark<SparkMax> max(int id, double gearRatio) {
		return new ChargerSpark<>(new SparkMax(id, MotorType.kBrushless), gearRatio);
	}
	
	public static ChargerSpark<SparkFlex> flex(int id, double gearRatio) {
		return new ChargerSpark<>(new SparkFlex(id, MotorType.kBrushless), gearRatio);
	}
	
	public static ChargerSpark<SparkMax> brushed(int id, double gearRatio) {
		return new ChargerSpark<>(new SparkMax(id, MotorType.kBrushed), gearRatio);
	}
	
	private ChargerSpark(BaseMotor baseMotor, double gearRatio) {
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
	
	public ChargerSpark<BaseMotor> configure(
		SparkBaseConfig config,
		SparkBase.ResetMode resetMode,
		SparkBase.PersistMode persistMode
	) {
		baseMotor.configure(config, resetMode, persistMode);
		return this;
	}
	
	public ChargerSpark<BaseMotor> configure(SparkBaseConfig config) {
		baseMotor.configure(config, kResetSafeParameters, kPersistParameters);
		return this;
	}
	
	@Override
	public Encoder encoder() { return encoder; }
	
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
	public void setPositionPID(PIDConstants constants) {
		var config = new SparkMaxConfig();
		config.closedLoop.p(constants.kP, kSlot0).i(constants.kI, kSlot0).d(constants.kD, kSlot0);
		baseMotor.configure(config, kNoResetSafeParameters, kPersistParameters);
	}
	
	@Override
	public void setVelocityPID(PIDConstants constants) {
		var config = new SparkMaxConfig();
		config.closedLoop.p(constants.kP, kSlot1).i(constants.kI, kSlot1).d(constants.kD, kSlot1);
		baseMotor.configure(config, kNoResetSafeParameters, kPersistParameters);
	}
	
	@Override
	public void enableContinuousInput() {
		var config = new SparkMaxConfig();
		config.closedLoop.positionWrappingEnabled(true).positionWrappingInputRange(-PI, PI);
		baseMotor.configure(config, kNoResetSafeParameters, kPersistParameters);
	}
	
	@Override
	public void close(){ baseMotor.close(); }
}
