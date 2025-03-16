package frc.chargers.hardware.motorcontrol;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.chargers.hardware.encoders.Encoder;
import frc.chargers.utils.Tracer;
import org.jetbrains.annotations.Nullable;

import java.util.Optional;

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
	protected final SparkBase baseApi;
	protected RelativeEncoder relativeEncoder;
	protected final SparkClosedLoopController pidController;
	protected SparkBaseConfig initialConfig;
	protected Encoder encoder = new Encoder() {
		@Override
		public double positionRad() {
			return rotationsToRadians(relativeEncoder.getPosition());
		}
		
		@Override
		public double velocityRadPerSec() {
			return rotationsPerMinuteToRadiansPerSecond(relativeEncoder.getVelocity());
		}
		
		@Override
		public void setPositionReading(Angle angle) {
			relativeEncoder.setPosition(angle.in(Rotations));
		}
	};
	
	public enum Model {
		SPARK_MAX, SPARK_FLEX
	}
	
	public ChargerSpark(int id, Model model, @Nullable SparkBaseConfig config) {
		this.baseApi = model == Model.SPARK_MAX ? new SparkMax(id, MotorType.kBrushless) : new SparkFlex(id, MotorType.kBrushless);
		this.relativeEncoder = baseApi.getEncoder();
		this.pidController = baseApi.getClosedLoopController();
		if (config != null) {
			this.initialConfig = config;
			tryUntilOk(baseApi, () -> baseApi.configure(config, kResetSafeParameters, kPersistParameters));
		} else {
			this.initialConfig = model == Model.SPARK_MAX ? new SparkMaxConfig() : new SparkFlexConfig();
		}
	}
	
	public ChargerSpark withAbsoluteEncoder() {
		var absoluteEncoder = baseApi.getAbsoluteEncoder();
		initialConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
		tryUntilOk(baseApi, () -> baseApi.configure(initialConfig, kResetSafeParameters, kPersistParameters));
		encoder = new Encoder() {
			@Override
			public double positionRad() {
				return rotationsToRadians(absoluteEncoder.getPosition());
			}
			
			@Override
			public double velocityRadPerSec() {
				return rotationsToRadians(absoluteEncoder.getVelocity());
			}
			
			@Override
			public void setPositionReading(Angle angle) {
				DriverStation.reportError("Position reading cannot be set for absolute encoder.", false);
			}
		};
		return this;
	}
	
	public ChargerSpark withSim(SimDynamics dynamics, DCMotor motorType) {
		if (!RobotBase.isSimulation()) return this;
		var sim = new SparkSim(baseApi, motorType);
		HAL.registerSimPeriodicAfterCallback(() -> {
			double batteryVoltage = RobotController.getBatteryVoltage();
			dynamics.acceptVolts().accept(MathUtil.clamp(sim.getAppliedOutput() * batteryVoltage, -12, 12));
			sim.iterate(radiansPerSecondToRotationsPerMinute(dynamics.velocity().getAsDouble()), batteryVoltage, 0.02);
		});
		return this;
	}
	
	public ChargerSpark enableAlternateEncoder() {
		if (baseApi instanceof SparkMax m) {
			relativeEncoder = m.getAlternateEncoder();
		} else if (baseApi instanceof SparkFlex m) {
			relativeEncoder = m.getExternalEncoder();
		} else {
			new Alert("Invalid spark max type -- code issue lmao", AlertType.kError).set(true);
		}
		return this;
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
			kSlot1, ffVolts
		);
	}
	
	@Override
	public void moveToPosition(double positionRads, double ffVolts) {
		pidController.setReference(
			radiansToRotations(positionRads),
			SparkBase.ControlType.kPosition,
			kSlot0, ffVolts
		);
	}
	
	@Override
	public void setCoastMode(boolean enabled) {
		Tracer.startTrace("set coast mode(spark)");
		baseApi.configure(
			initialConfig.idleMode(enabled ? SparkBaseConfig.IdleMode.kCoast : SparkBaseConfig.IdleMode.kBrake),
			kNoResetSafeParameters,
			kPersistParameters
		);
		Tracer.endTrace();
	}
	
	@Override
	public void setControlsConfig(ControlsConfig newConfig) {
		if (newConfig.positionPID().kP != 0.0) {
			initialConfig.closedLoop
				.pid(newConfig.positionPID().kP * (2 * PI), newConfig.positionPID().kI * (2 * PI), newConfig.positionPID().kD * (2 * PI), kSlot0)
				.positionWrappingEnabled(newConfig.continuousInput())
				.positionWrappingInputRange(-PI, PI);
		}
		if (newConfig.velocityPID().kP != 0.0) {
			initialConfig.closedLoop
				.pid(newConfig.velocityPID().kP * (2 * PI), newConfig.velocityPID().kI * (2 * PI), newConfig.velocityPID().kD * (2 * PI), kSlot1);
		}
		if (newConfig.gearRatio() != 1.0) {
			initialConfig.encoder
				.positionConversionFactor(1 / newConfig.gearRatio())
				.velocityConversionFactor(1 / newConfig.gearRatio());
		}
		tryUntilOk(baseApi, () -> baseApi.configure(initialConfig, kResetSafeParameters, kPersistParameters));
	}
	
	@Override
	public void close() {
		if (RobotBase.isSimulation()) baseApi.close();
	}
}
