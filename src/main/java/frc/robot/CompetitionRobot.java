package frc.robot;

import choreo.auto.AutoChooser;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.commands.WheelRadiusCharacterization.Direction;
import frc.robot.components.GyroWrapper;
import frc.robot.components.OperatorUi;
import frc.robot.components.RobotVisualization;
import frc.robot.components.vision.AprilTagVision;
import frc.robot.constants.TargetPoses;
import frc.robot.constants.Setpoint;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CoralIntakePivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.SwerveConfigurator;
import frc.robot.subsystems.swerve.SwerveDrive;
import monologue.LogLocal;
import monologue.Monologue;
import org.ironmaple.simulation.SimulatedArena;

import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.*;
import static frc.chargers.utils.TriggerUtil.bind;
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
	
	private final InputStream forwardOutput =
		InputStream.of(driverController::getLeftY)
			.negate()
			.deadband(0.1, 1);
	private final InputStream strafeOutput =
		InputStream.of(driverController::getLeftX)
			.negate()
			.deadband(0.1, 1);
	private final InputStream rotationOutput =
		InputStream.of(driverController::getRightX)
			.negate()
			.deadband(0.1, 1);
	private final InputStream manualElevatorInput =
		InputStream.of(manualOverrideController::getLeftY)
			.times(-0.7)
			.signedPow(2);
	private final InputStream manualPivotInput =
		InputStream.of(manualOverrideController::getRightY)
			.times(-0.3)
			.signedPow(1.3);
	
	public CompetitionRobot() {
		// Required for ChargerTalonFX and ChargerCANcoder to work
		StatusSignalRefresher.startPeriodic(this);
		// calls runTcp() and setups jni
		LaserCanUtil.setup(true);
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
								targetPoses.reefBlue[wantedPathTarget]
							)
						);
				}
			} else {
				driverController.cross()
					.and(operatorUi.targetLevelIs(wantedLevel))
					.whileTrue(botCommands.moveTo(Setpoint.score(wantedLevel)));
			}
		}
		
		driverController.square()
			.whileTrue(botCommands.sourceIntakeWithAim(targetPoses.eastSourceBlue, forwardOutput, strafeOutput));
		driverController.circle()
			.whileTrue(botCommands.sourceIntakeWithAim(targetPoses.westSourceBlue, forwardOutput, strafeOutput));
		
		driverController.R1()
			.whileTrue(coralIntake.outtakeForeverCmd());
		driverController.L1()
			.whileTrue(botCommands.moveTo(Setpoint.STOW_LOW));
		
		// anti-tipping
		gyroWrapper.isTipping
			.onTrue(botCommands.moveTo(Setpoint.STOW_LOW));
		
		/* Manual override controller bindings */
		var manualCtrlAllowed = operatorUi.isManualOverride.and(teleop());
		
		manualCtrlAllowed
			.whileTrue(elevator.setPowerCmd(manualElevatorInput))
			.whileTrue(coralIntakePivot.setPowerCmd(manualPivotInput));
		manualCtrlAllowed
			.and(manualOverrideController.leftBumper())
			.whileTrue(coralIntake.intakeForeverCmd());
		manualCtrlAllowed
			.and(manualOverrideController.rightBumper())
			.whileTrue(coralIntake.outtakeForeverCmd());
	}
	
	private void mapDefaultCommands() {
		drivetrain.setDefaultCommand(
			drivetrain.driveCmd(forwardOutput, strafeOutput, rotationOutput, false)
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
			() -> new WheelRadiusCharacterization(drivetrain, Direction.COUNTER_CLOCKWISE)
		);
		testModeChooser.addCmd("Elevator characterization", elevator::sysIdCmd);
		testModeChooser.addCmd("Drive FF Characterization", drivetrain::sysIdCmd);
		testModeChooser.addCmd("Reset odo test", () -> Commands.runOnce(drivetrain::resetToDemoPose));
		testModeChooser.addCmd("Align", () -> drivetrain.alignCmd(new Pose2d(5,7, Rotation2d.kZero), false));
		
		SmartDashboard.putData("TestChooser", testModeChooser);
		test().onTrue(testModeChooser.selectedCommandScheduler().ignoringDisable(true));
	}
}