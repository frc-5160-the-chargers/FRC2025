package frc.robot;

import choreo.auto.AutoChooser;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.chargers.utils.InputStream;
import frc.chargers.utils.StatusSignalRefresher;
import frc.chargers.utils.TunableValues;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.RobotCommands;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CoralIntakePivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.SwerveConfigurator;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.vision.AprilTagVision;
import monologue.ExtrasLogger;
import monologue.LogLocal;
import monologue.Monologue;
import org.ironmaple.simulation.SimulatedArena;

import static edu.wpi.first.units.Units.*;
import static frc.chargers.utils.UtilMethods.configureDefaultLogging;
import static monologue.Monologue.GlobalLog;

@Logged
public class Robot extends TimedRobot implements LogLocal {
	private final SwerveDrive drivetrain = new SwerveDrive(SwerveConfigurator.DEFAULT_DRIVE_CONFIG);
	private final CoralIntake coralIntake = new CoralIntake();
	private final CoralIntakePivot coralIntakePivot = new CoralIntakePivot();
	private final Elevator elevator = new Elevator();
	private final AprilTagVision vision = new AprilTagVision(this);
	
	@NotLogged private final CommandPS5Controller driverController = new CommandPS5Controller(0);
	
	// logging doesnt really work for sendables
	@NotLogged private final AutoChooser autoChooser = new AutoChooser();
	
	@NotLogged private final SwerveSetpointGenerator setpointGen = drivetrain.createSetpointGenerator(
		KilogramSquareMeters.of(6.283),
		Amps.of(60)
	);
	
	private final RobotCommands botCommands = new RobotCommands(
		drivetrain, coralIntake, coralIntakePivot, elevator, setpointGen
	);
	private final AutoCommands autoCommands = new AutoCommands(
		botCommands, drivetrain.createAutoFactory(),
		coralIntake, elevator
	);
	
	public Robot() {
		// Required for ChargerTalonFX and ChargerCANcoder to work
		StatusSignalRefresher.startPeriodic(this);
		// logging setup(required)
		Epilogue.bind(this);
		Monologue.setup(this, Epilogue.getConfig());
		configureDefaultLogging(Epilogue.getConfig());
		logMetadata();
		// enables tuning mode
		TunableValues.setTuningMode(true);
		
		// enable high-frequency odometry
		if (!RobotBase.isSimulation()) {
			addPeriodic(drivetrain::updateOdometry, 1 / SwerveConfigurator.ODOMETRY_FREQUENCY_HZ);
		}
		
		mapTriggers();
		mapDefaultCommands();
		mapAutoModes();
		// Vision setup - there are 2 overloads for addVisionData
		vision.setGlobalEstimateConsumer(drivetrain::addVisionData);
		vision.setSingleTagEstimateConsumer(drivetrain::addVisionData);
		vision.setSimPoseSupplier(drivetrain::bestPose);
		if (RobotBase.isSimulation()) {
			SimulatedArena.getInstance().placeGamePiecesOnField();
			drivetrain.resetPose(new Pose2d(5, 7, Rotation2d.kZero));
		}
		log("hasInitialized", true);

		autoChooser.addCmd("figure eight", autoCommands::figureEight);
	}
	
	@Override
	public void robotPeriodic() {
		// All of this code is required
		var startTime = System.nanoTime();
		CommandScheduler.getInstance().run();
		if (RobotBase.isSimulation()) {
			SimulatedArena.getInstance().simulationPeriodic();
			log("simulatedCoralPositions", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
			log("simulatedAlgaePositions", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
		}
		log("loopRuntime", (System.nanoTime() - startTime) / 1e6);
	}
	
	private void mapTriggers() {
		// TODO
	}
	
	private void mapDefaultCommands() {
		drivetrain.setDefaultCommand(
			drivetrain.driveCmd(
				InputStream.of(driverController::getLeftY)
					.negate()
					.log("driverController/xOutput"),
				InputStream.of(driverController::getLeftX)
					.negate()
					.log("driverController/yOutput"),
				InputStream.of(driverController::getRightX)
					.negate()
					.log("driverController/rotationOutput"),
				true
			)
		);
		//elevator.setDefaultCommand(elevator.stopCmd());
		// TODO - Set other default commands here
	}
	
	private void logMetadata() {
		GlobalLog.logMetadata("GitDate", BuildConstants.GIT_DATE);
		GlobalLog.logMetadata("BuildDate", BuildConstants.BUILD_DATE);
		GlobalLog.logMetadata("GitBranch", BuildConstants.GIT_BRANCH);
		GlobalLog.logMetadata("GitDirty", Integer.toString(BuildConstants.DIRTY));
		GlobalLog.logMetadata("GitSHA", BuildConstants.GIT_SHA);
		ExtrasLogger.start(this, new PowerDistribution());
	}
	
	private void mapAutoModes() {
		// TODO
		autoChooser.addCmd("MultiPieceCenter", autoCommands::multiPieceCenter);
		SmartDashboard.putData("AutoChooser", autoChooser);
		if (RobotBase.isSimulation()) {
			autoChooser.addCmd("MoveUp", () -> elevator.moveToHeightCmd(Meters.of(1.2)));
			autoChooser.addCmd(
				"PathfindTest",
				() -> drivetrain.pathfindCmd(new Pose2d(15, 4, Rotation2d.kZero), false, null)
			);
			autoChooser.addCmd(
				"SingleTagEstimationTest",
				() -> Commands.runOnce(() -> drivetrain.enableSingleTagEstimation(19))
			);
		}
		RobotModeTriggers.autonomous().onTrue(autoChooser.selectedCommandScheduler());
	}
}