package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
import frc.robot.subsystems.swerve.SwerveDrive.SwerveDriveConfig;

import java.util.List;

import static edu.wpi.first.units.Units.*;
import static org.ironmaple.simulation.drivesims.COTS.WHEELS.DEFAULT_NEOPRENE_TREAD;

public class SwerveConfigurator {
	private SwerveConfigurator(){}

	public static SwerveDriveConfig defaultConfig() {
		return new SwerveDriveConfig(
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
				new PIDConstants(5.0, 0.0, 0.0) // path rotation pid
			),
			Rotation2d::new, // Dummy gyro angle supplier because sim only
			getRealDriveMotors(), // real drive motor getter method
			getRealTurnMotors(),  // real turn motor getter method
			List.of() // encoders
		);
	}

	private static List<Motor> getRealTurnMotors() {
		return List.of(
			ChargerSpark.max(6, null),
			ChargerSpark.max(7, null),
			ChargerSpark.max(8, null),
			ChargerSpark.max(9, null)
		);
	}

	private static List<Motor> getRealDriveMotors() {
		var tl = new ChargerTalonFX(0, null);
		var tr = new ChargerTalonFX(1, null);
		var bl = new ChargerTalonFX(2, null);
		var br = new ChargerTalonFX(3, null);

		for (var motor: List.of(tl, tr, bl, br)) {
			motor.getConfigurator().apply(
				new CurrentLimitsConfigs()
					.withStatorCurrentLimit(50)
					.withSupplyCurrentLimit(90)
					.withStatorCurrentLimitEnable(true)
					.withSupplyCurrentLimitEnable(true)
			);
		}

		return List.of(tl, tr, bl, br);
	}
}
