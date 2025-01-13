package frc.robot.subsystems.swerve;

import frc.chargers.hardware.encoders.VoidEncoder;
import frc.chargers.utils.PIDConstants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.chargers.hardware.motorcontrol.ChargerSpark;
import frc.chargers.hardware.motorcontrol.ChargerTalonFX;
import frc.chargers.hardware.motorcontrol.Motor;
import frc.robot.subsystems.swerve.SwerveDrive.ControlsConfig;
import frc.robot.subsystems.swerve.SwerveDrive.HardwareConfig;
import frc.robot.subsystems.swerve.SwerveDrive.ModuleType;
import frc.robot.subsystems.swerve.SwerveDrive.SwerveCorner;
import frc.robot.subsystems.swerve.SwerveDrive.SwerveDriveConfig;

import static edu.wpi.first.units.Units.*;
import static org.ironmaple.simulation.drivesims.COTS.WHEELS.DEFAULT_NEOPRENE_TREAD;

public class SwerveConfigurator {
	private SwerveConfigurator(){}

	public static final SwerveDriveConfig DEFAULT_CONFIG =
		new SwerveDriveConfig(
			new HardwareConfig(
				Inches.of(27), // trackwidth
				Inches.of(27), // wheelbase
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
				0.0 // kT
			),
			Rotation2d::new, // Dummy gyro angle supplier because sim only
			SwerveConfigurator::getSteerMotor,
			SwerveConfigurator::getDriveMotor,
			corner -> new VoidEncoder()
		);
	
	private static Motor getSteerMotor(SwerveCorner corner) {
		var id = switch (corner) {
			case TOP_LEFT -> 0;
			case TOP_RIGHT -> 1;
			case BOTTOM_LEFT -> 2;
			case BOTTOM_RIGHT -> 3;
		};
		return ChargerSpark.max(id, null);
	}
	
	private static Motor getDriveMotor(SwerveCorner corner) {
		var id = switch (corner) {
			case TOP_LEFT -> 0;
			case TOP_RIGHT -> 1;
			case BOTTOM_LEFT -> 2;
			case BOTTOM_RIGHT -> 3;
		};
		return new ChargerTalonFX(id, null);
	}
}
