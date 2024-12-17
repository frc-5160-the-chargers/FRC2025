package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.chargers.utils.InputStream;
import frc.chargers.utils.UtilExtensionMethods;
import frc.chargers.utils.UtilMethods;
import frc.robot.subsystems.swerve.SwerveConfigurator;
import frc.robot.subsystems.swerve.SwerveDrive;
import lombok.experimental.ExtensionMethod;
import org.ironmaple.simulation.SimulatedArena;

@Logged(strategy = Logged.Strategy.OPT_IN)
@ExtensionMethod(UtilExtensionMethods.class)
public class DriverPracticeSim extends TimedRobot {
	@Logged private final SwerveDrive drivetrainOne = createSimBot(
		"drivetrainOne",
		new Pose2d(5.0, 7.0, Rotation2d.kZero)
	);
	
	private int currControllerId = 0;
	private SwerveDrive createSimBot(String name, Pose2d initialPose) {
		var drivetrain = new SwerveDrive(name, SwerveConfigurator.getDefaultConfig());
		drivetrain.resetPose(initialPose);
		
		var controller = new CommandXboxController(currControllerId);
		currControllerId++;
		
		drivetrain.setDefaultCommand(
			drivetrain.teleopDriveCmd(
				InputStream.of(controller::getLeftY)
					.negate()
					.log("driverController" + currControllerId + "/xOutput"),
				InputStream.of(controller::getLeftX)
					.negate()
					.log("driverController" + currControllerId + "/yOutput"),
				InputStream.of(controller::getRightX)
					.negate()
					.log("driverController" + currControllerId + "/rotationOutput"),
				true
			)
		);
		return drivetrain;
	}
	
	public DriverPracticeSim() {
		// logging config; do not remove
		Epilogue.bind(this);
		UtilMethods.configureDefaultLogging();
	}
	
	private static final LinearFilter voltageFilter = LinearFilter.movingAverage(200);
	
	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		if (RobotBase.isSimulation()) {
			SimulatedArena.getInstance().simulationPeriodic();
			// occasionally, maplesim can output NaN battery voltages
			RoboRioSim.setVInVoltage(
				voltageFilter.calculate(
					MathUtil.clamp(RobotController.getBatteryVoltage(), 0, 12)
				)
			);
		}
	}
}
