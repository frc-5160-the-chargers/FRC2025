package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.chargers.hardware.motorcontrol.MotorIO;
import frc.robot.subsystems.swerve.SwerveDrive.ControlsConfig;
import frc.robot.subsystems.swerve.SwerveDrive.HardwareConfig;
import frc.robot.subsystems.swerve.SwerveDrive.ModuleType;
import frc.robot.subsystems.swerve.SwerveDrive.SwerveDriveConfig;

import java.util.List;

import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import static edu.wpi.first.units.Units.*;
import static org.ironmaple.simulation.drivesims.SwerveModuleSimulation.WHEEL_GRIP.DEFAULT_NEOPRENE_TREAD;

public class SwerveConfigurator {
	private SwerveConfigurator(){}
	
	public static SwerveDriveConfig getDefaultConfig() {
		return new SwerveDriveConfig(
			new HardwareConfig(
				Inches.of(27), // trackwidth
				Inches.of(27), // wheelbase
				Inches.of(3), // width of bumpers
				DCMotor.getKrakenX60(1),
				DCMotor.getNEO(1),
				MetersPerSecond.of(4.5),
				DEFAULT_NEOPRENE_TREAD.cof, // coefficient of friction,
				Kilograms.of(45) // mass
			),
			ModuleType.MK4iL2,
			new ControlsConfig(
				new PIDConstants(15.0, 0.0, 0.01),
				new PIDConstants(0.5, 0.0, 0.01),
				new SimpleMotorFeedforward(0.03, 0.13),
				new PIDConstants(5.0, 0.0, 0.0),
				new PIDConstants(5.0, 0.0, 0.0)
			),
			Rotation2d::new, // Dummy gyro angle supplier because sim only
			SwerveConfigurator::getTurnMotors, // real drive motor getter method
			SwerveConfigurator::getDriveMotors,  // real turn motor getter method
			List.of() // encoders
		);
	}
	
	private static List<MotorIO> getTurnMotors(double gearRatio) {
		var tl = new SparkMax(6, kBrushless);
		var tr = new SparkMax(6, kBrushless);
		var bl = new SparkMax(6, kBrushless);
		var br = new SparkMax(6, kBrushless);
		
		return List.of(
			MotorIO.of(tl, gearRatio),
			MotorIO.of(tr, gearRatio),
			MotorIO.of(bl, gearRatio),
			MotorIO.of(br, gearRatio)
		);
	}
	
	private static List<MotorIO> getDriveMotors(double gearRatio) {
		var tl = new TalonFX(0);
		var tr = new TalonFX(0);
		var bl = new TalonFX(0);
		var br = new TalonFX(0);
		
		return List.of(
			MotorIO.of(tl, gearRatio),
			MotorIO.of(tr, gearRatio),
			MotorIO.of(bl, gearRatio),
			MotorIO.of(br, gearRatio)
		);
	}
}
