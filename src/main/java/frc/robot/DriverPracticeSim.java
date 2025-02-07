package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.chargers.utils.InputStream;
import frc.chargers.utils.StatusSignalRefresher;
import frc.chargers.utils.UtilMethods;
import frc.robot.subsystems.swerve.SwerveConfigurator;
import frc.robot.subsystems.swerve.SwerveDrive;
import lombok.experimental.ExtensionMethod;
import monologue.LogLocal;
import monologue.Monologue;
import org.ironmaple.simulation.SimulatedArena;

import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.test;
import static frc.chargers.utils.UtilMethods.configureDefaultLogging;

@ExtensionMethod(UtilMethods.class)
public class DriverPracticeSim extends TimedRobot implements LogLocal {
	@Logged public final SwerveDrive drivetrainOne = createSimBot(
		new Pose2d(5.0, 7.0, Rotation2d.kZero)
	);

	private int currControllerId = 0;
	private SwerveDrive createSimBot(Pose2d initialPose) {
		var drivetrain = new SwerveDrive(SwerveConfigurator.DEFAULT_DRIVE_CONFIG);
		drivetrain.resetPose(initialPose);

		var controller = new CommandXboxController(currControllerId);
		currControllerId++;

		drivetrain.setDefaultCommand(
			drivetrain.driveCmd(
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
		// Required for ChargerTalonFX and ChargerCANcoder to work
		StatusSignalRefresher.startPeriodic(this);
		// logging config; do not remove
		Epilogue.bind(this);
		Monologue.setup(this, Epilogue.getConfig());
		configureDefaultLogging(Epilogue.getConfig());

		SimulatedArena.getInstance().placeGamePiecesOnField();
		DriverStation.silenceJoystickConnectionWarning(true);
		test().whileTrue(
			drivetrainOne.pathfindCmd(() -> new Pose2d(5.26, 5.21, Rotation2d.fromDegrees(-121.34)), null)
		);
	}

	@Override
	public void robotPeriodic() {
		var startTime = System.nanoTime();
		RoboRioSim.setVInVoltage(12);
		CommandScheduler.getInstance().run();
		if (RobotBase.isSimulation()) {
			SimulatedArena.getInstance().simulationPeriodic();
			log("simulatedCoralPositions", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
			log("simulatedAlgaePositions", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
		}
		log("loopRuntime", (System.nanoTime() - startTime) / 1e6);
	}
}
