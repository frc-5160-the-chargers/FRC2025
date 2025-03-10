package frc.robot;

import choreo.auto.AutoChooser;
import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.epilogue.logging.FileBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.chargers.utils.InputStream;
import frc.chargers.utils.LaserCanUtil;
import frc.chargers.utils.StatusSignalRefresher;
import frc.chargers.utils.TunableValues;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.RobotCommands;
import frc.robot.commands.SimulatedAutoEnder;
import frc.robot.components.GyroWrapper;
import frc.robot.components.OperatorUi;
import frc.robot.components.vision.AprilTagVision;
import frc.robot.constants.BuildConstants;
import frc.robot.constants.Setpoint;
import frc.robot.constants.TargetPoses;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CoralIntakePivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.SwerveConfigurator;
import frc.robot.subsystems.swerve.SwerveDrive;
import monologue.ExtrasLogger;
import monologue.LogLocal;
import monologue.Monologue;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.urcl.URCL;

import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.autonomous;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.test;
import static frc.chargers.utils.TriggerUtil.bind;
import static frc.chargers.utils.TriggerUtil.doubleClicked;
import static frc.robot.constants.OtherConstants.*;
import static monologue.Monologue.GlobalLog;

@Logged
public class CompetitionRobot extends TimedRobot implements LogLocal {
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
	private final Climber climber = new Climber();
	
	/* Generic constants/utility classes */
	private final RobotVisualization visualizer =
		new RobotVisualization(drivetrain, coralIntake, coralIntakePivot, elevator);
	private final SwerveSetpointGenerator setpointGen = drivetrain.createSetpointGenerator(
		SwerveConfigurator.BODY_MOI, SwerveConfigurator.DRIVE_CURRENT_LIMIT
	);
	private final TargetPoses targetPoses =
		new TargetPoses(REEF_SCORE_OFFSET, SOURCE_OFFSET, SwerveConfigurator.HARDWARE_SPECS);
	
	/* Commands */
	@NotLogged private final RobotCommands botCommands =
		new RobotCommands(drivetrain, coralIntake, coralIntakePivot, elevator, setpointGen);
	@NotLogged private final AutoCommands autoCommands =
		new AutoCommands(botCommands, drivetrain.createAutoFactory(), coralIntake, drivetrain, targetPoses);
	
	/* Auto choosers */
	@NotLogged private final AutoChooser autoChooser = new AutoChooser();
	@NotLogged private final AutoChooser testModeChooser = new AutoChooser();
	
	/* Controllers/Driver input */
	private final CommandPS5Controller driverController = new CommandPS5Controller(DRIVER_CONTROLLER_PORT);
	private final OperatorUi operatorUi = new OperatorUi();
	private final CommandXboxController manualOverrideController = new CommandXboxController(MANUAL_CONTROLLER_PORT);
	
	private final InputStream slowModeOutput =
		InputStream.of(driverController::getL2Axis)
			.deadband(0.1, 1)
			.map(it -> 1 - it / 2);
	private final InputStream forwardOutput =
		InputStream.of(driverController::getLeftY)
			.deadband(0.1, 1)
			.times(slowModeOutput)
			.negate();
	private final InputStream strafeOutput =
		InputStream.of(driverController::getLeftX)
			.deadband(0.1, 1)
			.times(slowModeOutput)
			.negate();
	private final InputStream rotationOutput =
		InputStream.of(driverController::getRightX)
			.deadband(0.1, 1)
			.times(slowModeOutput)
			.negate();
	private final InputStream manualElevatorInput =
		InputStream.of(manualOverrideController::getLeftY)
			.deadband(0.1, 1)
			.times(-0.5)
			.signedPow(2);
	private final InputStream manualPivotInput =
		InputStream.of(manualOverrideController::getRightY)
			.times(-0.3);
	
	public CompetitionRobot() {
		// Required for ChargerTalonFX and ChargerCANcoder to work
		StatusSignalRefresher.startPeriodic(this);
		// calls runTcp() and setups jni
		LaserCanUtil.setup(true);
		// logging setup(required)
		Epilogue.bind(this);
		Epilogue.getConfig().backend = EpilogueBackend.multi(
			new FileBackend(DataLogManager.getLog()),
			new NTEpilogueBackend(NetworkTableInstance.getDefault())
		);
		Monologue.setup(this, Epilogue.getConfig());
		GlobalLog.enableCommandLogging();
		logMetadata();
		DataLogManager.logNetworkTables(false);
		URCL.start();
		SignalLogger.start();
		ExtrasLogger.start(this, null);
		// enables tuning mode
		TunableValues.setTuningMode(true);
		
		mapTriggers();
		mapDefaultCommands();
		mapAutoModes();
		mapTestCommands();
		
		addPeriodic(drivetrain::updateOdometry, 1 / SwerveConfigurator.ODOMETRY_FREQUENCY_HZ);
		// Vision setup - there are 2 overloads for addVisionData
		vision.setGlobalEstimateConsumer(drivetrain::addVisionData);
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
			for (int wantedPathTarget = 0; wantedPathTarget < 12; wantedPathTarget++) {
				var moveAndPathfindCmd = botCommands.pathfindAndMoveTo(
					Setpoint.score(wantedLevel),
					targetPoses.reefBlue[wantedPathTarget]
				);
				var moveAndAimCmd = botCommands.aimAndMoveTo(
					Setpoint.score(wantedLevel), targetPoses.reefBlue[wantedPathTarget],
					forwardOutput, strafeOutput
				);
				driverController.cross()
					.and(operatorUi.targetLevelIs(wantedLevel))
					.and(operatorUi.pathfindTargetIs(wantedPathTarget))
					.whileTrue(USE_PATHFINDING ? moveAndPathfindCmd : moveAndAimCmd);
			}
		}
		
		driverController.square()
			.whileTrue(botCommands.aimAndSourceIntake(targetPoses.eastSourceBlue, forwardOutput, strafeOutput));
		driverController.circle()
			.whileTrue(botCommands.aimAndSourceIntake(targetPoses.westSourceBlue, forwardOutput, strafeOutput));
		
		driverController.L1()
			.whileTrue(botCommands.moveTo(Setpoint.STOW_LOW));
		driverController.R2()
			.whileTrue(coralIntake.outtakeForeverCmd());
		
		driverController.povUp()
			.whileTrue(drivetrain.driveCmd(() -> NUDGE_OUTPUT, () -> 0, () -> 0, false));
		driverController.povDown()
			.whileTrue(drivetrain.driveCmd(() -> -NUDGE_OUTPUT, () -> 0, () -> 0, false));
		driverController.povLeft()
			.whileTrue(drivetrain.driveCmd(() -> 0, () -> -NUDGE_OUTPUT, () -> 0, false));
		driverController.povRight()
			.whileTrue(drivetrain.driveCmd(() -> 0, () -> NUDGE_OUTPUT, () -> 0, false));
		
		// anti-tipping
		gyroWrapper.isTipping
			.onTrue(
				botCommands.moveTo(Setpoint.STOW_LOW)
					.withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
			);
		
		operatorUi.isManualOverride
			.whileTrue(elevator.setPowerCmd(manualElevatorInput))
			.whileTrue(coralIntakePivot.setPowerCmd(manualPivotInput));
		
		manualOverrideController.povUp()
			.and(operatorUi.isManualOverride)
			.whileTrue(coralIntake.outtakeForeverCmd());
		manualOverrideController.povDown()
			.and(operatorUi.isManualOverride)
			.whileTrue(coralIntake.intakeForeverCmd());
		
		manualOverrideController.rightBumper()
			.and(operatorUi.isManualOverride)
			.whileTrue(
				botCommands.moveTo(Setpoint.INTAKE)
					.alongWith(coralIntake.intakeForeverCmd())
					.withName("Manual source intake")
			);
		manualOverrideController.leftBumper()
			.and(operatorUi.isManualOverride)
			.whileTrue(botCommands.moveTo(Setpoint.STOW_LOW));
		
		manualOverrideController.rightTrigger()
			.and(operatorUi.isManualOverride)
			.whileTrue(climber.climbUp());
		manualOverrideController.leftTrigger()
			.and(operatorUi.isManualOverride)
			.whileTrue(climber.climbDown());
		
		manualOverrideController.a()
			.and(operatorUi.isManualOverride)
			.whileTrue(botCommands.moveTo(Setpoint.score(1)));
		manualOverrideController.b()
			.and(operatorUi.isManualOverride)
			.whileTrue(botCommands.moveTo(Setpoint.score(2)));
		manualOverrideController.y()
			.and(operatorUi.isManualOverride)
			.whileTrue(botCommands.moveTo(Setpoint.score(3)));
		manualOverrideController.x()
			.and(operatorUi.isManualOverride)
			.whileTrue(botCommands.moveTo(Setpoint.score(4)));
		
		doubleClicked(manualOverrideController.start())
			.onTrue(Commands.runOnce(drivetrain::resetToDemoPose).ignoringDisable(true));
		doubleClicked(manualOverrideController.back())
			.onTrue(coralIntakePivot.resetAngleToZeroCmd());
	}
	
	private void mapDefaultCommands() {
		drivetrain.setDefaultCommand(
			drivetrain.driveCmd(forwardOutput, strafeOutput, rotationOutput, false)
		);
		elevator.setDefaultCommand(elevator.idleCmd());
		coralIntake.setDefaultCommand(coralIntake.idleCmd());
		coralIntakePivot.setDefaultCommand(coralIntakePivot.setPowerCmd(manualPivotInput));
		climber.setDefaultCommand(climber.idleCmd());
	}
	
	private void logMetadata() {
		GlobalLog.logMetadata("GitDate", BuildConstants.GIT_DATE);
		GlobalLog.logMetadata("BuildDate", BuildConstants.BUILD_DATE);
		GlobalLog.logMetadata("GitBranch", BuildConstants.GIT_BRANCH);
		GlobalLog.logMetadata("GitDirty", Integer.toString(BuildConstants.DIRTY));
		GlobalLog.logMetadata("GitSHA", BuildConstants.GIT_SHA);
	}
	
	private void mapAutoModes() {
		// TODO
		autoChooser.addCmd("3x L4 Right", autoCommands::tripleL4South);
		autoChooser.addCmd("4x L1 Right", autoCommands::quadL1South);
		autoChooser.addCmd("L4 L1 L1 Right", autoCommands::l4L1L1South);
		autoChooser.addCmd("L4 L4 L1 Right", autoCommands::l4L4L1South);
		autoChooser.addCmd("One Piece L4", autoCommands::onePieceL4);
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
		testModeChooser.addCmd("MoveToCoralSetpoint", () -> coralIntakePivot.setPowerCmd(() -> 1));
		testModeChooser.addCmd(
			"Pathfind",
			() -> drivetrain.pathfindCmd(targetPoses.reefBlue[5], true, setpointGen)
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
			drivetrain::wheelRadiusCharacterization
		);
		testModeChooser.addCmd("Reset odo test", () -> Commands.runOnce(drivetrain::resetToDemoPose));
		testModeChooser.addCmd("Align", () -> drivetrain.alignCmd(new Pose2d(5,7, Rotation2d.kZero), false));
		
		SmartDashboard.putData("TestChooser", testModeChooser);
		test().onTrue(testModeChooser.selectedCommandScheduler().ignoringDisable(true));
	}
}