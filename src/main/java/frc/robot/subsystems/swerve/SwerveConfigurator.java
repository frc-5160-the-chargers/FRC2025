package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.chargers.hardware.encoders.ChargerCANcoder;
import frc.chargers.hardware.motorcontrol.ChargerTalonFX;
import frc.chargers.utils.PIDConstants;
import frc.robot.subsystems.swerve.SwerveDrive.ControlsConfig;
import frc.robot.subsystems.swerve.SwerveDrive.HardwareConfig;
import frc.robot.subsystems.swerve.SwerveDrive.ModuleType;
import frc.robot.subsystems.swerve.SwerveDrive.PoseEstimationMode;
import frc.robot.subsystems.swerve.SwerveDrive.SwerveCorner;
import frc.robot.subsystems.swerve.SwerveDrive.SwerveDriveConfig;

import static edu.wpi.first.units.Units.*;
import static org.ironmaple.simulation.drivesims.COTS.WHEELS.DEFAULT_NEOPRENE_TREAD;

public class SwerveConfigurator {
	private SwerveConfigurator(){}
	
	public static final Current DRIVE_CURRENT_LIMIT = Amps.of(90);
	public static final Current DRIVE_STATOR_CURRENT_LIMIT = Amps.of(120);
	public static final MomentOfInertia BODY_MOI = KilogramSquareMeters.of(6.883);
	public static final double ODOMETRY_FREQUENCY_HZ = 200;
	private static final Pigeon2 PIGEON = new Pigeon2(0);
	
	public static final SwerveDriveConfig DEFAULT_DRIVE_CONFIG =
		new SwerveDriveConfig(
			new HardwareConfig(
				Inches.of(27), // trackwidth
				Inches.of(32), // wheelbase
				DCMotor.getKrakenX60(1), // drive motor type
				DCMotor.getNEO(1), // turn motor type
				MetersPerSecond.of(4.5), // max linear speed
				DEFAULT_NEOPRENE_TREAD.cof, // coefficient of friction,
				Kilograms.of(45) // mass
			),
			ModuleType.SwerveX2L2P11,
			new ControlsConfig(
				new PIDConstants(2, 0.0, 0.0), // azimuth pid
				new PIDConstants(2.0, 0.0, 0.01), // velocity pid
				new SimpleMotorFeedforward(0.032, 2.73), // velocity feedforward
				new PIDConstants(5.0, 0.0, 0.0), // path translation pid
				new PIDConstants(5.0, 0.0, 0.0), // path rotation pid,
				0.0, // kT
				PoseEstimationMode.AUTOMATIC
				// SELF_RUN means that you have to call updateOdometry periodically yourself
			),
			PIGEON::getRotation2d,
			// Class::new is basically shorthand for creating a function object that takes in
			// the class's constructor parameters and returns an instance of the class.
			// Here, it would be Function<SwerveCorner, Motor>.
			RealDriveMotor::new,
			RealSteerMotor::new,
			RealSteerEncoder::new,
			null, // sim steer motor config
			null
//			new TalonFXConfiguration().withCurrentLimits(
//				new CurrentLimitsConfigs()
//					.withSupplyCurrentLimit(DRIVE_CURRENT_LIMIT)
//					.withSupplyCurrentLimitEnable(true)
//					.withStatorCurrentLimit(DRIVE_STATOR_CURRENT_LIMIT)
//					.withStatorCurrentLimitEnable(true)
//			) // sim drive motor config
		);
	
	private static class RealDriveMotor extends ChargerTalonFX {
		public RealDriveMotor(SwerveCorner corner) {
			super(getId(corner), true, getConfig(corner));
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
		
		private static TalonFXConfiguration getConfig(SwerveCorner corner) {
			var config = new TalonFXConfiguration();
			config.CurrentLimits
				.withStatorCurrentLimit(DRIVE_STATOR_CURRENT_LIMIT)
				.withStatorCurrentLimitEnable(true)
				.withSupplyCurrentLimit(DRIVE_CURRENT_LIMIT)
				.withSupplyCurrentLimitEnable(true);
			return config;
		}
	}
	
	private static class RealSteerMotor extends ChargerTalonFX {
		public RealSteerMotor(SwerveCorner corner) {
			super(getId(corner), true, getConfig(corner));
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
		
		private static TalonFXConfiguration getConfig(SwerveCorner corner) {
			var config = new TalonFXConfiguration();
			config.CurrentLimits
				.withStatorCurrentLimit(60)
				.withStatorCurrentLimitEnable(true)
				.withSupplyCurrentLimit(60)
				.withSupplyCurrentLimitEnable(true);
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
				case TOP_LEFT -> Radians.of(-1.376);
				case TOP_RIGHT -> Radians.of(0.801);
				case BOTTOM_LEFT -> Radians.of(0.919);
				case BOTTOM_RIGHT -> Radians.of(1.028);
			};
			config.MagnetSensor.withMagnetOffset(offset);
			return config;
		}
		
		public RealSteerEncoder(SwerveCorner corner) {
			super(getId(corner), true, getConfig(corner));
		}
	}
}
