package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.chargers.hardware.encoders.VoidEncoder;
import frc.chargers.hardware.motorcontrol.ChargerSpark;
import frc.chargers.hardware.motorcontrol.ChargerSpark.SparkModel;
import frc.chargers.hardware.motorcontrol.ChargerTalonFX;
import frc.chargers.hardware.motorcontrol.Motor;
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
	public static final MomentOfInertia BODY_MOI = KilogramSquareMeters.of(6.883);
	
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
			ModuleType.MK4iL2,
			new ControlsConfig(
				new PIDConstants(7.5, 0.0, 0.0), // azimuth pid
				new PIDConstants(2.0, 0.0, 0.01), // velocity pid
				new SimpleMotorFeedforward(0.032, 2.73), // velocity feedforward
				new PIDConstants(5.0, 0.0, 0.0), // path translation pid
				new PIDConstants(5.0, 0.0, 0.0), // path rotation pid,
				0.0, // kT
				PoseEstimationMode.AUTOMATIC
			),
			Rotation2d::new, // Dummy gyro angle supplier because sim only
			SwerveConfigurator::getSteerMotor,
			SwerveConfigurator::getDriveMotor,
			corner -> new VoidEncoder(),
			null,
			new TalonFXConfiguration().withCurrentLimits(
				new CurrentLimitsConfigs()
					.withSupplyCurrentLimit(90)
					.withSupplyCurrentLimitEnable(true)
					.withStatorCurrentLimit(100)
					.withStatorCurrentLimitEnable(true)
			)
		);
	
	private static Motor getSteerMotor(SwerveCorner corner) {
		var id = switch (corner) {
			case TOP_LEFT -> 0;
			case TOP_RIGHT -> 1;
			case BOTTOM_LEFT -> 2;
			case BOTTOM_RIGHT -> 3;
		};
		return new ChargerSpark(id, SparkModel.SPARK_MAX, null);
	}
	
	private static Motor getDriveMotor(SwerveCorner corner) {
		var id = switch (corner) {
			case TOP_LEFT -> 0;
			case TOP_RIGHT -> 1;
			case BOTTOM_LEFT -> 2;
			case BOTTOM_RIGHT -> 3;
		};
		var config = new TalonFXConfiguration();
		config.CurrentLimits.StatorCurrentLimitEnable = true;
		config.CurrentLimits.StatorCurrentLimit = DRIVE_CURRENT_LIMIT.in(Amps);
		return new ChargerTalonFX(id, true, config);
	}
}
