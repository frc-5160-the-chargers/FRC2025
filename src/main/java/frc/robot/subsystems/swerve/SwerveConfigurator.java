package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.chargers.hardware.encoders.ChargerCANcoder;
import frc.chargers.hardware.motorcontrol.ChargerTalonFX;
import frc.chargers.utils.PIDConstants;
import frc.robot.subsystems.swerve.SwerveDrive.ModuleType;
import frc.robot.subsystems.swerve.SwerveDrive.SwerveControlsConfig;
import frc.robot.subsystems.swerve.SwerveDrive.SwerveCorner;
import frc.robot.subsystems.swerve.SwerveDrive.SwerveHardwareSpecs;
import frc.robot.subsystems.swerve.SwerveDrive.SwerveMotorConfig;

import static edu.wpi.first.units.Units.*;
import static org.ironmaple.simulation.drivesims.COTS.WHEELS.DEFAULT_NEOPRENE_TREAD;

public class SwerveConfigurator {
	private SwerveConfigurator(){}
	
	public static final Current DRIVE_CURRENT_LIMIT = Amps.of(110);
	public static final Current DRIVE_STATOR_CURRENT_LIMIT = Amps.of(120);
	public static final MomentOfInertia BODY_MOI = KilogramSquareMeters.of(6.883);
	public static final double ODOMETRY_FREQUENCY_HZ = 200;
	
	public static final ModuleType MODULE_TYPE = ModuleType.SwerveX2L2P11;
	public static final SwerveHardwareSpecs HARDWARE_SPECS =
		new SwerveHardwareSpecs(
			Inches.of(29.375), // trackwidth(width)
			Inches.of(35.375), // wheelbase(height)
			DCMotor.getKrakenX60(1), // drive motor type
			DCMotor.getKrakenX60(1), // turn motor type
			MetersPerSecond.of(4.5), // max linear speed
			DEFAULT_NEOPRENE_TREAD.cof, // coefficient of friction,
			Pounds.of(120), // mass
			Inches.of(2) // width of bumpers
		);
	public static final SwerveControlsConfig CONTROLS_CONFIG =
		new SwerveControlsConfig(
			new PIDConstants(8, 0.001, 0.2), // azimuth pid
			new PIDConstants(5.0, 0, 0), // velocity pid
			new SimpleMotorFeedforward(0, 0.128), // velocity feedforward
			new PIDConstants(10.0, 0, 0), // path translation pid
			new PIDConstants(10.0, 0, 0), // path rotation pid,
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
			)
		);
	
	private static class RealDriveMotor extends ChargerTalonFX {
		public RealDriveMotor(SwerveCorner corner) {
			super(getId(corner), true, getConfig());
			//super.positionSignal.setUpdateFrequency(ODOMETRY_FREQUENCY_HZ);
		}
		
		private static int getId(SwerveCorner corner) {
			return switch (corner) {
				case TOP_LEFT -> 4;
				case TOP_RIGHT -> 6;
				case BOTTOM_LEFT -> 2;
				case BOTTOM_RIGHT -> 3;
			};
		}
		
		private static TalonFXConfiguration getConfig() {
			var config = new TalonFXConfiguration();
			config.CurrentLimits
				.withStatorCurrentLimit(DRIVE_STATOR_CURRENT_LIMIT)
				.withStatorCurrentLimitEnable(true)
				.withSupplyCurrentLimit(DRIVE_CURRENT_LIMIT)
				.withSupplyCurrentLimitEnable(true);
			config.MotorOutput
				.withNeutralMode(NeutralModeValue.Brake)
				.withInverted(InvertedValue.CounterClockwise_Positive);
			return config;
		}
	}
	
	private static class RealSteerMotor extends ChargerTalonFX {
		public RealSteerMotor(SwerveCorner corner) {
			super(getId(corner), true, getConfig());
			//super.positionSignal.setUpdateFrequency(ODOMETRY_FREQUENCY_HZ);
		}
		
		private static int getId(SwerveCorner corner) {
			return switch (corner) {
				case TOP_LEFT -> 5;
				case TOP_RIGHT -> 7;
				case BOTTOM_LEFT -> 1;
				case BOTTOM_RIGHT -> 8;
			};
		}
		
		private static TalonFXConfiguration getConfig() {
			var config = new TalonFXConfiguration();
			config.CurrentLimits
				.withStatorCurrentLimit(60)
				.withStatorCurrentLimitEnable(true)
				.withSupplyCurrentLimit(60)
				.withSupplyCurrentLimitEnable(true);
			config.MotorOutput
				.withInverted(InvertedValue.Clockwise_Positive)
				.withNeutralMode(NeutralModeValue.Brake);
			return config;
		}
	}
	
	private static class RealSteerEncoder extends ChargerCANcoder {
		public static int getId(SwerveCorner corner) {
			return switch (corner) {
				case TOP_LEFT -> 3;
				case TOP_RIGHT -> 2;
				case BOTTOM_LEFT -> 1;
				case BOTTOM_RIGHT -> 4;
			};
		}
		
		private static CANcoderConfiguration getConfig(SwerveCorner corner) {
			var config = new CANcoderConfiguration();
			var offset = switch (corner) {
				case TOP_LEFT -> Radians.of(-0.63).plus(Degrees.of(-45));
				case TOP_RIGHT -> Radians.of(0).plus(Degrees.of(45));
				case BOTTOM_LEFT -> Radians.of(0.12).plus(Degrees.of(45));
				case BOTTOM_RIGHT -> Radians.of(-1.32).plus(Degrees.of(-45));
			};
			
			config.MagnetSensor
				.withMagnetOffset(offset)
				.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
			return config;
		}
		
		public RealSteerEncoder(SwerveCorner corner) {
			super(getId(corner), true, getConfig(corner));
		}
	}
}
