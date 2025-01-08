package frc.chargers.hardware.motorcontrol;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import frc.chargers.hardware.encoders.Encoder;
import lombok.Getter;
import org.jetbrains.annotations.Nullable;

import java.util.function.Consumer;

import static com.revrobotics.spark.ClosedLoopSlot.kSlot0;
import static com.revrobotics.spark.ClosedLoopSlot.kSlot1;
import static com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters;
import static com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters;
import static com.revrobotics.spark.SparkLowLevel.MotorType;
import static edu.wpi.first.math.util.Units.radiansToRotations;
import static edu.wpi.first.math.util.Units.rotationsToRadians;
import static edu.wpi.first.units.Units.Rotations;
import static java.lang.Math.PI;

public class ChargerSpark<BaseMotor extends SparkBase> implements Motor, AutoCloseable {
	@Getter private final BaseMotor baseMotor;
	private final RelativeEncoder baseEncoder;
	private final SparkClosedLoopController pidController;
	private final Encoder encoder = new Encoder() {
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
	
	public static ChargerSpark<SparkMax> max(int id, @Nullable Consumer<SparkMax> configureFn) {
		var baseMotor = new SparkMax(id, MotorType.kBrushless);
		if (configureFn != null) configureFn.accept(baseMotor);
		return new ChargerSpark<>(baseMotor);
	}
	
	public static ChargerSpark<SparkFlex> flex(int id, @Nullable Consumer<SparkFlex> configureFn) {
		var baseMotor = new SparkFlex(id, MotorType.kBrushless);
		if (configureFn != null) configureFn.accept(baseMotor);
		return new ChargerSpark<>(baseMotor);
	}
	
	public static ChargerSpark<SparkMax> brushed(int id, @Nullable Consumer<SparkMax> configureFn) {
		var baseMotor = new SparkMax(id, MotorType.kBrushed);
		if (configureFn != null) configureFn.accept(baseMotor);
		return new ChargerSpark<>(baseMotor);
	}
	
	/**
	 * Use {@link ChargerSpark#max} or {@link ChargerSpark#flex} instead.
	 */
	private ChargerSpark(BaseMotor baseMotor) {
		this.baseMotor = baseMotor;
		this.baseEncoder = baseMotor.getEncoder();
		this.pidController = baseMotor.getClosedLoopController();
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
			radiansToRotations(velocityRadPerSec),
			SparkBase.ControlType.kVelocity,
			kSlot1
		);
	}
	
	@Override
	public void setTorqueCurrent(double currentAmps) {}
	
	@Override
	public void moveToPosition(double positionRads, double ffVolts) {
		pidController.setReference(
			radiansToRotations(positionRads),
			SparkBase.ControlType.kVelocity,
			kSlot0
		);
	}
	
	@Override
	public void setCommonConfig(CommonConfig newConfig) {
		var baseConfig = new SparkMaxConfig();
		if (newConfig.positionPID().kP != 0.0) {
			baseConfig.closedLoop
				.pid(newConfig.positionPID().kP, newConfig.positionPID().kI, newConfig.positionPID().kD, kSlot0)
				.positionWrappingEnabled(newConfig.continuousInput())
				.positionWrappingInputRange(-PI, PI);
		}
		if (newConfig.velocityPID().kP != 0.0) {
			baseConfig.closedLoop
				.pid(newConfig.velocityPID().kP, newConfig.velocityPID().kI, newConfig.velocityPID().kD, kSlot1);
		}
		if (newConfig.gearRatio() != 1.0) {
			baseConfig.encoder
				.positionConversionFactor(1 / newConfig.gearRatio())
				.velocityConversionFactor(1 / newConfig.gearRatio());
		}
		baseMotor.configure(baseConfig, kNoResetSafeParameters, kPersistParameters);
	}
	
	@Override
	public void close() { baseMotor.close(); }
}
