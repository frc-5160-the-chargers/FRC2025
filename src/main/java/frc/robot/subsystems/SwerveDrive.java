package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.chargers.utils.swerve.SwerveDriveBase;

import java.util.List;

import static edu.wpi.first.units.Units.*;
import static org.ironmaple.simulation.drivesims.SwerveModuleSimulation.WHEEL_GRIP.DEFAULT_NEOPRENE_TREAD;

// note: do not put @Logged onto this, it bricks things
public class SwerveDrive extends SwerveDriveBase {
	private static final AHRS navX = new AHRS();
	private static final SwerveDriveConfig config =
		new SwerveDriveConfig(
			new HardwareConfig(
				Inches.of(27), // trackwidth
				Inches.of(27), // wheelbase
				Inches.of(3), // width of bumpers
				DCMotor.getKrakenX60(1),
				DCMotor.getNEO(1),
				MetersPerSecond.of(4.5),
				DEFAULT_NEOPRENE_TREAD.cof // coefficient of friction
			),
			ModuleType.MK4iL2,
			new ControlsConfig(
				new PIDConstants(15.0, 0.0, 0.01),
				new PIDConstants(0.5, 0.0, 0.01),
				new SimpleMotorFeedforward(0.03, 0.13),
				new PIDConstants(5.0, 0.0, 0.0),
				new PIDConstants(5.0, 0.0, 0.0)
			),
			navX::getRotation2d,
			gearRatio -> List.of(), // real drive motor getter method
			gearRatio -> List.of(),  // real turn motor getter method,
			List.of()
		);
	
	public SwerveDrive(String logName) {
		super(logName, config);
	}
}
