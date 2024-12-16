package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.config.SparkMaxConfig;
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
	
	public static SwerveDriveConfig getDefaultConfig() {
		return new SwerveDriveConfig(
			new HardwareConfig(
				Inches.of(27), // trackwidth
				Inches.of(27), // wheelbase
				DCMotor.getKrakenX60(1), // drive motor type
				DCMotor.getNEO(1), // turn motor type
				MetersPerSecond.of(4.5), // max linear speed
				DEFAULT_NEOPRENE_TREAD.cof, // coefficient of friction,
				Kilograms.of(45), // mass
				KilogramSquareMeters.of(6.883) // robot MOI
			),
			ModuleType.MK4iL2,
			new ControlsConfig(
				new PIDConstants(15.0, 0.0, 0.01), // azimuth pid
				new PIDConstants(0.5, 0.0, 0.01), // velocity pid
				new SimpleMotorFeedforward(0.03, 0.13), // velocity feedforward
				new PIDConstants(5.0, 0.0, 0.0), // path translation pid
				new PIDConstants(5.0, 0.0, 0.0) // path rotation pid
			),
			Rotation2d::new, // Dummy gyro angle supplier because sim only
			SwerveConfigurator::getTurnMotors, // real drive motor getter method
			SwerveConfigurator::getDriveMotors,  // real turn motor getter method
			List.of() // encoders
		);
	}
	
	private static List<Motor> getTurnMotors(double gearRatio) {
		return List.of(
			ChargerSpark.max(6, gearRatio).configure(new SparkMaxConfig()),
			ChargerSpark.max(6, gearRatio),
			ChargerSpark.max(6, gearRatio),
			ChargerSpark.max(6, gearRatio)
		);
	}
	
	private static List<Motor> getDriveMotors(double gearRatio) {
		var tl = new ChargerTalonFX(0, gearRatio);
		var tr = new ChargerTalonFX(0, gearRatio);
		var bl = new ChargerTalonFX(0, gearRatio);
		var br = new ChargerTalonFX(0, gearRatio);
		
		for (var motor: List.of(tl, tr, bl, br)) {
			motor.getConfigurator().apply(
				new CurrentLimitsConfigs()
					.withStatorCurrentLimit(50)
					.withSupplyCurrentLimit(80)
			);
		}
		
		return List.of(tl, tr, bl, br);
	}
}
