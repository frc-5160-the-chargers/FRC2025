package frc.robot;

import choreo.auto.AutoChooser;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.chargers.utils.InputStream;
import frc.chargers.utils.StatusSignalRefresher;
import frc.chargers.utils.TunableValues;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.RobotCommands;
import frc.robot.commands.SimulatedAutoEnder;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.commands.WheelRadiusCharacterization.Direction;
import frc.robot.components.GyroWrapper;
import frc.robot.components.OperatorUi;
import frc.robot.components.vision.AprilTagVision;
import frc.robot.constants.PathfindingPoses;
import frc.robot.constants.Setpoint;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CoralIntakePivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.SwerveConfigurator;
import frc.robot.subsystems.swerve.SwerveDrive;
import monologue.ExtrasLogger;
import monologue.LogLocal;
import monologue.Monologue;
import org.ironmaple.simulation.SimulatedArena;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.*;
import static frc.chargers.utils.TriggerUtil.bind;
import static monologue.Monologue.GlobalLog;

@Logged
public class CompetitionRobot extends TimedRobot implements LogLocal {
	private static final boolean USE_PATHFINDING = true;
	
	/* Subsystems/Components */
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
	private final AprilTagVision vision = new AprilTagVision();
	
	/* Generic constants/utility classes */
	private final RobotVisualization visualizer =
		new RobotVisualization(drivetrain, coralIntake, coralIntakePivot, elevator);
	private final SwerveSetpointGenerator setpointGen = drivetrain.createSetpointGenerator(
		KilogramSquareMeters.of(6.283),  // robot moi
		Amps.of(60) // drive current limit
	);
	private final PathfindingPoses pathfindingPoses = new PathfindingPoses(
		new Translation2d(Inches.of(-15), Inches.of(-7.5)), // reef offset
		new Translation2d(Meters.of(-0.13), Meters.of(-0.5)), // north source offset,
		new Translation2d(Meters.of(-0.13), Meters.of(0.5)), // south source offset
		SwerveConfigurator.HARDWARE_SPECS
	);
	
	/* Commands */
	@NotLogged private final RobotCommands botCommands = new RobotCommands(
		drivetrain, coralIntake, coralIntakePivot,
		elevator, setpointGen
	);
	@NotLogged private final AutoCommands autoCommands = new AutoCommands(
		botCommands, drivetrain.createAutoFactory(), coralIntake,
		elevator, drivetrain, pathfindingPoses
	);
	
	/* Auto choosers */
	@NotLogged private final AutoChooser autoChooser = new AutoChooser();
	@NotLogged private final AutoChooser testModeChooser = new AutoChooser();
	
	/* Controllers/Driver input */
	private final CommandPS5Controller driverController = new CommandPS5Controller(0);
	private final OperatorUi operatorUi = new OperatorUi();
	private final CommandXboxController manualOverrideController = new CommandXboxController(1);
	
	public CompetitionRobot() {
		// Required for ChargerTalonFX and ChargerCANcoder to work
		StatusSignalRefresher.startPeriodic(this);
		// logging setup(required)
		Epilogue.bind(this);
		Monologue.setup(this, Epilogue.getConfig());
		GlobalLog.enableCommandLogging();
		logMetadata();
		DataLogManager.start();
		// enables tuning mode
		TunableValues.setTuningMode(true);
		
		mapTriggers();
		mapDefaultCommands();
		mapAutoModes();
		mapTestCommands();
		
		addPeriodic(drivetrain::updateOdometry, 0.02);
		// Vision setup - there are 2 overloads for addVisionData
		vision.setGlobalEstimateConsumer(drivetrain::addVisionData);
		//vision.setSingleTagEstimateConsumer(drivetrain::addVisionData);
		vision.setSimPoseSupplier(drivetrain::bestPose);
		DriverStation.silenceJoystickConnectionWarning(true);
		SmartDashboard.putData(
			"View Connection warnings",
			Commands.runOnce(() -> DriverStation.silenceJoystickConnectionWarning(false))
		);
		
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
		visualizer.periodic();
		vision.periodic();
		operatorUi.periodic();
		log("loopRuntime", (System.nanoTime() - startTime) / 1e6);
	}
	
	private void mapTriggers() {
		bind(
			new Alert("Driver controller not connected", AlertType.kWarning),
			() -> !driverController.isConnected()
		);
		bind(
			new Alert("Operator override not connected", AlertType.kWarning),
			() -> !manualOverrideController.isConnected()
		);
		
		/* Driver controller/Operator UI bindings */
		for (int wantedLevel = 1; wantedLevel <= 4; wantedLevel++) {
			if (USE_PATHFINDING) {
				for (int wantedPathTarget = 0; wantedPathTarget < 12; wantedPathTarget++) {
					driverController.cross()
						.and(operatorUi.targetLevelIs(wantedLevel))
						.and(operatorUi.pathfindTargetIs(wantedPathTarget))
						.whileTrue(
							botCommands.pathfindAndMoveTo(
								Setpoint.score(wantedLevel),
								pathfindingPoses.reefBlue[wantedPathTarget]
							)
						);
				}
			} else {
				driverController.cross()
					.and(operatorUi.targetLevelIs(wantedLevel))
					.whileTrue(botCommands.moveTo(Setpoint.score(wantedLevel)));
			}
		}
		
		if (USE_PATHFINDING) {
			driverController.square()
				.whileTrue(drivetrain.pathfindCmd(pathfindingPoses.eastSourceBlue, true, setpointGen));
			driverController.circle()
				.whileTrue(drivetrain.pathfindCmd(pathfindingPoses.westSourceBlue, true, setpointGen));
		}
		
		driverController.R1()
			.whileTrue(coralIntake.outtakeForeverCmd());
		driverController.L1()
			.whileTrue(botCommands.moveTo(Setpoint.STOW_LOW));
		driverController.R2()
			.whileTrue(botCommands.sourceIntake());
		
		/* Manual override controller bindings */
		var elevatorInput =
			InputStream.of(manualOverrideController::getLeftY)
				.times(-0.7)
				.signedPow(2)
				.log("manualOverrideController/elevatorInput");
		var pivotInput =
			InputStream.of(manualOverrideController::getRightY)
				.times(-0.3)
				.signedPow(1.3)
				.log("manualOverrideController/pivotInput");
		operatorUi.isManualOverride
			.and(teleop())
			.whileTrue(elevator.setPowerCmd(elevatorInput))
			.whileTrue(coralIntakePivot.setPowerCmd(pivotInput));
	}
	
	private void mapDefaultCommands() {
		drivetrain.setDefaultCommand(
			drivetrain.driveCmd(
				InputStream.of(driverController::getLeftY)
					.negate()
					.deadband(0.1, 1)
					.log("driverController/xOutput"),
				InputStream.of(driverController::getLeftX)
					.negate()
					.deadband(0.1, 1)
					.log("driverController/yOutput"),
				InputStream.of(driverController::getRightX)
					.negate()
					.deadband(0.1, 1)
					.log("driverController/rotationOutput"),
				false
			)
		);
		elevator.setDefaultCommand(elevator.idleCmd());
		coralIntake.setDefaultCommand(coralIntake.idleCmd());
		coralIntakePivot.setDefaultCommand(coralIntakePivot.idleCmd());
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
		autoChooser.addCmd("3x L4 Right", autoCommands::tripleL4South);
		autoChooser.addCmd("4x L1 Right", autoCommands::quadL1South);
		autoChooser.addCmd("L4 L1 L1 Right", autoCommands::l4L1L1South);
		autoChooser.addCmd("L4 L4 L1 Right", autoCommands::l4L4L1South);
		autoChooser.addCmd("(TEST ONLY) figure eight", autoCommands::figureEight);
		autoChooser.addCmd("(TEST ONLY) simple path", autoCommands::pathTest);
		autoChooser.addCmd("(TEST ONLY) multi piece", autoCommands::multiPieceTest);
		
		SmartDashboard.putData("AutoChooser", autoChooser);
		autonomous()
			.onTrue(autoChooser.selectedCommandScheduler())
			.onTrue(new SimulatedAutoEnder());
	}
	
	private void mapTestCommands() {
		testModeChooser.addCmd("MoveToDemoSetpoint", botCommands::moveToDemoSetpoint);
		testModeChooser.addCmd(
			"Pathfind",
			() -> drivetrain.pathfindCmd(pathfindingPoses.reefBlue[5], true, setpointGen)
		);
		testModeChooser.addCmd(
			"Outtake",
			() -> coralIntake.setHasCoralInSimCmd(true).andThen(coralIntake.outtakeCmd())
		);
		testModeChooser.addCmd("Score L4", () -> botCommands.scoreSequence(4));
		if (RobotBase.isSimulation()) {
			testModeChooser.addCmd(
				"Stow and simulate coral",
				() -> coralIntake.setHasCoralInSimCmd(true).andThen(botCommands.moveTo(Setpoint.STOW_LOW))
			);
		}
		testModeChooser.addCmd("Move to L3", () -> botCommands.moveTo(Setpoint.score(3)));
		testModeChooser.addCmd(
			"Wheel radius characterization",
			() -> new WheelRadiusCharacterization(drivetrain, Direction.COUNTER_CLOCKWISE)
		);
		testModeChooser.addCmd("Elevator characterization", elevator::sysIdCmd);
		testModeChooser.addCmd("Drive FF Characterization", drivetrain::sysIdCmd);
		testModeChooser.addCmd("Reset odo test", autoCommands::resetOdometryTest);
		
		SmartDashboard.putData("TestChooser", testModeChooser);
		test().onTrue(testModeChooser.selectedCommandScheduler().ignoringDisable(true));
	}
}