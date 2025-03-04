package frc.chargers.hardware.motorcontrol;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import frc.chargers.hardware.encoders.Encoder;
import org.jetbrains.annotations.Nullable;

import static com.revrobotics.spark.ClosedLoopSlot.kSlot0;
import static com.revrobotics.spark.ClosedLoopSlot.kSlot1;
import static com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters;
import static com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters;
import static com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters;
import static com.revrobotics.spark.SparkLowLevel.MotorType;
import static edu.wpi.first.math.util.Units.*;
import static edu.wpi.first.units.Units.Rotations;
import static frc.chargers.utils.UtilMethods.tryUntilOk;
import static java.lang.Math.PI;

/**
 * A Spark Max/Spark Flex that implements the Motor interface.
 * To access more low-level capabilities of the base api, inherit from this class.
 */
public class ChargerSpark implements Motor {
	public final SparkBase baseApi;
	protected final RelativeEncoder baseEncoder;
	protected final SparkClosedLoopController pidController;
	protected SparkBaseConfig initialConfig;
	protected final Encoder encoder = new Encoder() {
		@Override
		public double positionRad() {
			return rotationsToRadians(baseEncoder.getPosition());
		}
		
		@Override
		public double velocityRadPerSec() {
			return rotationsPerMinuteToRadiansPerSecond(baseEncoder.getVelocity());
		}
		
		@Override
		public void setPositionReading(Angle angle) {
			baseEncoder.setPosition(angle.in(Rotations));
		}
	};
	
	/**
	 * A utility method to disable common unused signals.
	 * @param configs The spark configs to-apply.
	 */
	public static void optimizeBusUtilizationOn(SparkBaseConfig... configs) {
		for (var config: configs) {
			config.signals
				.analogPositionPeriodMs(65535)
				.analogVelocityPeriodMs(65535)
				.externalOrAltEncoderPosition(65535)
				.externalOrAltEncoderVelocity(65535)
				.analogVoltagePeriodMs(65535)
				.iAccumulationPeriodMs(65535);
		}
	}
	
	public enum Model {
		SPARK_MAX, SPARK_FLEX
	}
	
	public ChargerSpark(int id, Model model, @Nullable SparkBaseConfig config) {
		this.baseApi = model == Model.SPARK_MAX ? new SparkMax(id, MotorType.kBrushless) : new SparkFlex(id, MotorType.kBrushless);
		this.baseEncoder = baseApi.getEncoder();
		this.pidController = baseApi.getClosedLoopController();
		if (config != null) {
			this.initialConfig = config;
			tryUntilOk(baseApi, () -> baseApi.configure(config, kResetSafeParameters, kPersistParameters));
		} else {
			this.initialConfig = model == Model.SPARK_MAX ? new SparkMaxConfig() : new SparkFlexConfig();
		}
	}
	
	@Override
	public Encoder encoder() { return encoder; }
	
	@Override
	public double outputVoltage() {
		return baseApi.getAppliedOutput() * baseApi.getBusVoltage();
	}
	
	@Override
	public double statorCurrent() { return baseApi.getOutputCurrent(); }
	
	@Override
	public double tempCelsius() { return baseApi.getMotorTemperature(); }
	
	@Override
	public int id() {
		return baseApi.getDeviceId();
	}
	
	@Override
	public void setVoltage(double volts) { baseApi.setVoltage(volts); }
	
	@Override
	public void setVelocity(double velocityRadPerSec, double ffVolts) {
		pidController.setReference(
			radiansToRotations(velocityRadPerSec),
			SparkBase.ControlType.kVelocity,
			kSlot1
		);
	}
	
	@Override
	public void moveToPosition(double positionRads, double ffVolts) {
		pidController.setReference(
			radiansToRotations(positionRads),
			SparkBase.ControlType.kVelocity,
			kSlot0
		);
	}
	
	@Override
	public void setCoastMode(boolean enabled) {
		baseApi.configure(
			initialConfig.idleMode(enabled ? SparkBaseConfig.IdleMode.kCoast : SparkBaseConfig.IdleMode.kBrake),
			kNoResetSafeParameters,
			kPersistParameters
		);
	}
	
	@Override
	public void setControlsConfig(ControlsConfig newConfig) {
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
		tryUntilOk(baseApi, () -> baseApi.configure(baseConfig, kNoResetSafeParameters, kPersistParameters));
	}
	
	@Override
	public void close() {
		if (RobotBase.isSimulation()) baseApi.close();
	}
}
