package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotBase;
import frc.chargers.hardware.encoders.ChargerCANcoder;
import frc.chargers.hardware.encoders.VoidEncoder;
import frc.chargers.hardware.motorcontrol.ChargerTalonFX;
import frc.chargers.hardware.motorcontrol.VoidMotor;
import frc.chargers.utils.data.PIDConstants;
import frc.robot.subsystems.swerve.SwerveDrive.ModuleType;
import frc.robot.subsystems.swerve.SwerveDrive.SwerveControlsConfig;
import frc.robot.subsystems.swerve.SwerveDrive.SwerveCorner;
import frc.robot.subsystems.swerve.SwerveDrive.SwerveHardwareSpecs;
import frc.robot.subsystems.swerve.SwerveDrive.SwerveMotorConfig;

import static edu.wpi.first.units.Units.*;
import static org.ironmaple.simulation.drivesims.COTS.WHEELS.DEFAULT_NEOPRENE_TREAD;

public class SwerveConfigurator {
	private SwerveConfigurator(){}
	
	public static final Current DRIVE_CURRENT_LIMIT = Amps.of(80);
	public static final Current DRIVE_STATOR_CURRENT_LIMIT = Amps.of(120);
	public static final MomentOfInertia BODY_MOI = KilogramSquareMeters.of(5.883);
	public static final double ODOMETRY_FREQUENCY_HZ = 200;
	private static final boolean USE_REMOTE_CANCODER = true;
	public static final ModuleType MODULE_TYPE = ModuleType.SwerveX2L2P11;
	
	public static final SwerveHardwareSpecs HARDWARE_SPECS =
		new SwerveHardwareSpecs(
			Inches.of(27), // trackwidth(width)
			Inches.of(32.5), // wheelbase(height)
			DCMotor.getKrakenX60(1), // drive motor type
			DCMotor.getKrakenX60(1), // steer motor type
			MetersPerSecond.of(4.5), // max linear speed
			DEFAULT_NEOPRENE_TREAD.cof, // coefficient of friction,
			Pounds.of(116), // mass
			Inches.of(3.5) // width of bumpers
		);
	
	// VecBuilder.fill
	// Encoders - pose estimate (x, y, theta)
	// Vision cameras - pose estimate (x, y, theta)
	// distance from tag
	// Area of tag
	// Pose estimator
	
	public static final SwerveControlsConfig CONTROLS_CONFIG =
		new SwerveControlsConfig(
			new PIDConstants(9, 0, 0.01), // azimuth pid - don't add d to this, it makes things weird
			new PIDConstants(0.4, 0, 0), // velocity pid
			new SimpleMotorFeedforward( // velocity feedforward
				RobotBase.isSimulation() ? 0.015 : 0.17,
				1 / (HARDWARE_SPECS.driveMotorType().KvRadPerSecPerVolt / MODULE_TYPE.driveGearRatio)
			),
			new PIDConstants(10, 0, 0.1), // path translation pid
			new PIDConstants(10, 0, 0.1), // path rotation pid,
			0.0 // kT
		);
	
	public static final SwerveMotorConfig DEFAULT_MOTOR_CONFIG =
		new SwerveMotorConfig(
			RealDriveMotor::new,
			RealSteerMotor::new,
			RealSteerEncoder::new,
			null, // sim steer motor config
			new TalonFXConfiguration().withCurrentLimits(
				new CurrentLimitsConfigs()
					.withSupplyCurrentLimit(DRIVE_CURRENT_LIMIT)
					.withSupplyCurrentLimitEnable(true)
					.withStatorCurrentLimit(DRIVE_STATOR_CURRENT_LIMIT)
					.withStatorCurrentLimitEnable(true)
			) // sim drive motor config
		);
	
	public static final SwerveMotorConfig DISABLED_MOTOR_CONFIG =
		new SwerveMotorConfig(
			corner -> new VoidMotor(),
			corner -> new VoidMotor(),
			corner -> new VoidEncoder(),
			null, null
		);
	
	private static class RealDriveMotor extends ChargerTalonFX {
		public RealDriveMotor(SwerveCorner corner) {
			super(getId(corner), true, getConfig(corner));
			super.setPositionUpdateRate(ODOMETRY_FREQUENCY_HZ);
		}
		
		private static int getId(SwerveCorner corner) {
			return switch (corner) {
				case TOP_LEFT -> 6;
				case TOP_RIGHT -> 3;
				case BOTTOM_LEFT -> 8;
				case BOTTOM_RIGHT -> 2;
			};
		}
		
		private static TalonFXConfiguration getConfig(SwerveCorner corner) {
			var config = new TalonFXConfiguration();
			config.CurrentLimits
				.withStatorCurrentLimit(DRIVE_STATOR_CURRENT_LIMIT)
				.withStatorCurrentLimitEnable(true)
				.withSupplyCurrentLimit(DRIVE_CURRENT_LIMIT)
				.withSupplyCurrentLimitEnable(true);
			config.MotorOutput
				.withNeutralMode(NeutralModeValue.Brake)
				.withInverted(InvertedValue.CounterClockwise_Positive);
			if (corner != SwerveCorner.TOP_RIGHT) {
				config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
			}
			return config;
		}
	}
	
	private static class RealSteerMotor extends ChargerTalonFX {
		public RealSteerMotor(SwerveCorner corner) {
			super(getId(corner), true, getConfig(corner));
			super.setPositionUpdateRate(ODOMETRY_FREQUENCY_HZ);
		}
		
		private static int getId(SwerveCorner corner) {
			return switch (corner) {
				case TOP_LEFT -> 12;
				case TOP_RIGHT -> 11;
				case BOTTOM_LEFT -> 9;
				case BOTTOM_RIGHT -> 10;
			};
		}
		
		private static TalonFXConfiguration getConfig(SwerveCorner corner) {
			var config = new TalonFXConfiguration();
			config.CurrentLimits
				.withStatorCurrentLimit(60)
				.withStatorCurrentLimitEnable(true)
				.withSupplyCurrentLimit(60)
				.withSupplyCurrentLimitEnable(true);
			config.MotorOutput
				.withInverted(InvertedValue.Clockwise_Positive)
				.withNeutralMode(NeutralModeValue.Brake);
			if (USE_REMOTE_CANCODER) {
				config.Feedback
					.withFeedbackRemoteSensorID(RealSteerEncoder.getId(corner))
					.withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder);
			}
			return config;
		}
	}
	
	private static class RealSteerEncoder extends ChargerCANcoder {
		public RealSteerEncoder(SwerveCorner corner) {
			super(getId(corner), true, getConfig(corner));
			if (USE_REMOTE_CANCODER) super.setPositionUpdateRate(ODOMETRY_FREQUENCY_HZ);
		}
		
		public static int getId(SwerveCorner corner) {
			return switch (corner) {
				case TOP_LEFT -> 2;
				case TOP_RIGHT -> 4;
				case BOTTOM_LEFT -> 3;
				case BOTTOM_RIGHT -> 1;
			};
		}
		
		private static CANcoderConfiguration getConfig(SwerveCorner corner) {
			var config = new CANcoderConfiguration();
			var offset = switch (corner) {
				case TOP_LEFT -> Radians.of(2.361); //Radians.of(-0.044).plus(Degrees.of(-45));
				case TOP_RIGHT -> Radians.of(2.602); //Radians.of(-1.333).plus(Degrees.of(45));
				case BOTTOM_LEFT -> Radians.of(-2.961); //Radians.of(-0.595).plus(Degrees.of(45));
				case BOTTOM_RIGHT -> Radians.of(-0.709); //Radians.of(0.12).plus(Degrees.of(-45));
			};
			config.MagnetSensor
				.withMagnetOffset(offset)
				.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
			return config;
		}
	}
}
