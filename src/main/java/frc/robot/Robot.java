package frc.robot;

import choreo.auto.AutoChooser;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.chargers.utils.InputStream;
import frc.chargers.utils.StatusSignalRefresher;
import frc.chargers.utils.TunableValues;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.RobotCommands;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.commands.WheelRadiusCharacterization.Direction;
import frc.robot.constants.Setpoint;
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

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.autonomous;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.test;
import static monologue.Monologue.GlobalLog;

@Logged
public class Robot extends TimedRobot implements LogLocal {
	private final GyroWrapper gyroWrapper = new GyroWrapper();
	private final SwerveDrive drivetrain = new SwerveDrive(
		SwerveConfigurator.HARDWARE_SPECS,
		SwerveConfigurator.CONTROLS_CONFIG,
		SwerveConfigurator.MODULE_TYPE,
		SwerveConfigurator.DEFAULT_MOTOR_CONFIG,
		gyroWrapper::yaw
	);
	private final CoralIntake coralIntake = new CoralIntake();
	private final CoralIntakePivot coralIntakePivot = new CoralIntakePivot();
	private final Elevator elevator = new Elevator();
	private final AprilTagVision vision = new AprilTagVision(this);
	
	private final CommandPS5Controller driverController = new CommandPS5Controller(0);
	private final AutoChooser autoChooser = new AutoChooser();
	private final AutoChooser testModeChooser = new AutoChooser();
	private final SwerveSetpointGenerator setpointGen = drivetrain.createSetpointGenerator(
		KilogramSquareMeters.of(6.283),  // robot moi
		Amps.of(60) // drive current limit
	);
	private final RobotVisualization visualizer =
		new RobotVisualization(drivetrain, coralIntake, coralIntakePivot, elevator);
	
	@NotLogged private final RobotCommands botCommands =
		new RobotCommands(drivetrain, coralIntake, coralIntakePivot, elevator, setpointGen);
	@NotLogged private final AutoCommands autoCommands =
		new AutoCommands(botCommands, drivetrain.createAutoFactory(), coralIntake, elevator);
	
	public Robot() {
		// Required for ChargerTalonFX and ChargerCANcoder to work
		StatusSignalRefresher.startPeriodic(this);
		// logging setup(required)
		Epilogue.bind(this);
		Monologue.setup(this, Epilogue.getConfig());
		Monologue.enableCommandLogging();
		logMetadata();
		DataLogManager.start();
		// enables tuning mode
		TunableValues.setTuningMode(true);
		
		mapTriggers();
		mapDefaultCommands();
		mapAutoModes();
		mapTestCommands();
		// Vision setup - there are 2 overloads for addVisionData
		vision.setGlobalEstimateConsumer(drivetrain::addVisionData);
		//vision.setSingleTagEstimateConsumer(drivetrain::addVisionData);
		vision.setSimPoseSupplier(drivetrain::bestPose);
		if (RobotBase.isSimulation()) {
			SimulatedArena.getInstance().placeGamePiecesOnField();
			drivetrain.resetPose(new Pose2d(5, 7, Rotation2d.kZero));
		}
	}
	
	@Override
	public void robotPeriodic() {
		// All of this code is required
		var startTime = System.nanoTime();
		CommandScheduler.getInstance().run();
		visualizer.render();
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
				false
			)
		);
		elevator.setDefaultCommand(elevator.stopCmd());
		coralIntake.setDefaultCommand(coralIntake.stopCmd());
		coralIntakePivot.setDefaultCommand(coralIntakePivot.stopCmd());
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
		autoChooser.addCmd("multi piece center", autoCommands::multiPieceCenter);
		autoChooser.addCmd("figure eight", autoCommands::figureEight);
		
		SmartDashboard.putData("AutoChooser", autoChooser);
		autonomous()
			.onTrue(autoChooser.selectedCommandScheduler())
			.onTrue(Commands.waitSeconds(15.3).andThen(() -> DriverStationSim.setEnabled(false)))
			.onTrue(Commands.runOnce(() -> coralIntake.runContinuously = true))
			.onFalse(Commands.runOnce(() -> coralIntake.runContinuously = false));
	}
	
	private void mapTestCommands() {
		testModeChooser.addCmd("MoveToDemoSetpoint", botCommands::moveToDemoSetpoint);
		testModeChooser.addCmd(
			"PathfindTest",
			() -> drivetrain.pathfindCmd(new Pose2d(15, 4, Rotation2d.k180deg), false, setpointGen)
		);
		testModeChooser.addCmd(
			"SingleTagEstimationTest",
			() -> Commands.runOnce(() -> drivetrain.enableSingleTagEstimation(19))
		);
		testModeChooser.addCmd(
			"ScoreL4",
			() -> botCommands.scoreSequence(4)
		);
		testModeChooser.addCmd(
			"Stow",
			() -> botCommands.moveTo(Setpoint.STOW)
		);
		testModeChooser.addCmd(
			"SimulateHasCoral",
			() -> coralIntake.setHasCoralInSimCmd(true)
		);
		testModeChooser.addCmd(
			"WheelRadiusCharacterization",
			() -> new WheelRadiusCharacterization(drivetrain, Direction.COUNTER_CLOCKWISE)
		);
		
		SmartDashboard.putData("TestChooser", testModeChooser);
		test().onTrue(testModeChooser.selectedCommandScheduler().ignoringDisable(true));
	}
}