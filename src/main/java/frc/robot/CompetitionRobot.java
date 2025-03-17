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
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.chargers.utils.AllianceUtil;
import frc.chargers.utils.LaserCanUtil;
import frc.chargers.utils.Tracer;
import frc.chargers.utils.data.StatusSignalRefresher;
import frc.chargers.utils.data.TunableValues;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.RobotCommands;
import frc.robot.commands.SimulatedAutoEnder;
import frc.robot.components.GyroWrapper;
import frc.robot.components.controllers.DriverController;
import frc.robot.components.controllers.OperatorController;
import frc.robot.components.vision.AprilTagVision;
import frc.robot.constants.BuildConstants;
import frc.robot.constants.Setpoint;
import frc.robot.constants.TargetPoses;
import frc.robot.constants.TargetPoses.ReefSide;
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

/*
Reminder: There is a rio 2 only GC config on lines 39-44 of build.gradle.
Comment it out if we are running a rio 1.
 */
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
	private final Elevator elevator = new Elevator();
	private final CoralIntake coralIntake = new CoralIntake(() -> elevator.heightMeters() < 0.15);
	private final CoralIntakePivot coralIntakePivot = new CoralIntakePivot(() -> 0, coralIntake.hasCoral.debounce(0.7));
	private final Climber climber = new Climber();
	private final AprilTagVision vision = new AprilTagVision();
	
	/* Generic constants/utility classes */
	private final RobotVisualization visualizer =
		new RobotVisualization(drivetrain, coralIntake, coralIntakePivot, elevator);
	private final SwerveSetpointGenerator setpointGen =
		drivetrain.createSetpointGenerator(SwerveConfigurator.BODY_MOI, SwerveConfigurator.DRIVE_CURRENT_LIMIT);
	private final TargetPoses targetPoses =
		new TargetPoses(REEF_SCORE_OFFSET, SOURCE_OFFSET, SwerveConfigurator.HARDWARE_SPECS);
	
	/* Commands */
	@NotLogged private final RobotCommands botCommands =
		new RobotCommands(drivetrain, coralIntake, coralIntakePivot, elevator);
	@NotLogged private final AutoCommands autoCommands =
		new AutoCommands(botCommands, drivetrain.createAutoFactory(), coralIntake, drivetrain, targetPoses);
	
	/* Auto choosers */
	@NotLogged private final AutoChooser autoChooser = new AutoChooser();
	@NotLogged private final AutoChooser testModeChooser = new AutoChooser();
	
	/* Controllers/Driver input */
	private final DriverController driver = new DriverController();
	private final OperatorController operator = new OperatorController();
	
	public CompetitionRobot() {
		// calls runTcp() and setups jni
		LaserCanUtil.setup(true);
		// logging setup(required)
		Epilogue.bind(this);
		Monologue.setup(this, Epilogue.getConfig());
		DataLogManager.start();
		GlobalLog.enableCommandLogging();
		logMetadata();
		URCL.start();
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
		// Tracer.trace(...) runs the method while recording loop times
		Tracer.trace("cmd scheduler", CommandScheduler.getInstance()::run);
		Tracer.trace("mech visualizer", visualizer::periodic);
		Tracer.trace("CAN signal refresh", StatusSignalRefresher::periodic); // you must add this line
		Tracer.trace("vision", vision::periodic);
		if (RobotBase.isSimulation()) {
			Tracer.trace("maple sim", SimulatedArena.getInstance()::simulationPeriodic);
		}
	}
	
	private void mapTriggers() {
		bind(
			new Alert("Driver controller not connected", AlertType.kWarning),
			() -> !driver.isConnected()
		);
		bind(
			new Alert("Operator override not connected", AlertType.kWarning),
			() -> !operator.isConnected()
		);
		
		driver.L1()
			.whileTrue(
				drivetrain.pathfindCmd(
					() -> targetPoses.closestReefPose(ReefSide.LEFT, drivetrain.poseEstimate()),
					setpointGen
				)
			);
		driver.R1()
			.whileTrue(
				drivetrain.pathfindCmd(
					() -> targetPoses.closestReefPose(ReefSide.RIGHT, drivetrain.poseEstimate()),
					setpointGen
				)
			);
		
		driver.povUp()
			.whileTrue(drivetrain.driveCmd(() -> NUDGE_OUTPUT, () -> 0, () -> 0, false).withName("nudge"));
		driver.povDown()
			.whileTrue(drivetrain.driveCmd(() -> -NUDGE_OUTPUT, () -> 0, () -> 0, false).withName("nudge"));
		driver.povLeft()
			.whileTrue(drivetrain.driveCmd(() -> 0, () -> NUDGE_OUTPUT, () -> 0, false).withName("nudge"));
		driver.povRight()
			.whileTrue(drivetrain.driveCmd(() -> 0, () -> -NUDGE_OUTPUT, () -> 0, false).withName("nudge"));
		
		doubleClicked(driver.touchpad())
			.onTrue(
				Commands.runOnce(() -> {
					var currPose = drivetrain.bestPose();
					drivetrain.resetPose(new Pose2d(currPose.getX(), currPose.getY(), AllianceUtil.flipIfRed(Rotation2d.kZero)));
				})
					.ignoringDisable(true)
					.withName("zero heading")
			);
		
		// anti-tipping
		gyroWrapper.isTipping
			.onTrue(botCommands.stow().withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
		
		operator.povUp()
			.whileTrue(coralIntake.outtakeForeverCmd());
		operator.povDown()
			.whileTrue(coralIntake.intakeForeverCmd());
		
		operator.rightBumper()
			.whileTrue(
				botCommands.moveTo(Setpoint.INTAKE)
					.alongWith(coralIntake.intakeForeverCmd())
					.withName("Manual source intake")
			);
		operator.leftBumper()
			.whileTrue(botCommands.stow());
		
		operator.a()
			.whileTrue(botCommands.moveTo(Setpoint.score(1)));
		operator.b()
			.whileTrue(botCommands.moveTo(Setpoint.score(2)));
		operator.y()
			.whileTrue(botCommands.moveTo(Setpoint.score(3)));
		operator.x()
			.whileTrue(botCommands.moveTo(Setpoint.score(4)));
		
		doubleClicked(operator.start())
			.onTrue(Commands.runOnce(() -> drivetrain.resetPose(drivetrain.getDemoPose())).ignoringDisable(true).unless(DriverStation::isAutonomous));
		doubleClicked(operator.back())
			.onTrue(climber.resetStartingAngle());
		operator.start()
			.whileTrue(botCommands.moveTo(Setpoint.ALGAE_PREP_L2));
		operator.back()
			.whileTrue(botCommands.moveTo(Setpoint.ALGAE_PREP_L3));
	}
	
	private void mapDefaultCommands() {
		drivetrain.setDefaultCommand(
			drivetrain.driveCmd(driver.forwardOutput, driver.strafeOutput, driver.rotationOutput, true)
		);
		elevator.setDefaultCommand(elevator.setPowerCmd(operator.manualElevatorInput));
		coralIntake.setDefaultCommand(coralIntake.idleCmd());
		coralIntakePivot.setDefaultCommand(coralIntakePivot.setPowerCmd(operator.manualPivotInput));
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
		autoChooser.addCmd("L1 + L4", autoCommands::l1L4);
		autoChooser.addCmd("Taxi", autoCommands::taxi);
		autoChooser.addCmd("3x L4 Right", autoCommands::tripleL4South);
		autoChooser.addCmd("4x L1 Right", autoCommands::quadL1South);
		autoChooser.addCmd("L4 L1 L1 Right", autoCommands::l4L1L1South);
		autoChooser.addCmd("L4 L4 L1 Right", autoCommands::l4L4L1South);
		autoChooser.addCmd("One Piece L4", autoCommands::onePieceL4);
		autoChooser.addCmd("One Piece L1", autoCommands::onePieceL1);
		autoChooser.addCmd("(TEST ONLY) figure eight", autoCommands::figureEight);
		autoChooser.addCmd("(TEST ONLY) multi piece", autoCommands::multiPieceTest);
		autoChooser.select("Taxi");
		
		SmartDashboard.putData("AutoChooser", autoChooser);
		autonomous()
			.onTrue(autoChooser.selectedCommandScheduler())
			.onTrue(new SimulatedAutoEnder());
	}
	
	private void mapTestCommands() {
		testModeChooser.addCmd("Simple Path", autoCommands::simplePath);
		testModeChooser.addCmd("Simple Path w/ rotate", autoCommands::simplePathWithRotate);
		
		testModeChooser.addCmd("MoveL4", () -> botCommands.moveTo(Setpoint.score(4)));
		testModeChooser.addCmd("MoveToDemoSetpoint", botCommands::moveToDemoSetpoint);
		testModeChooser.addCmd("MoveToCoralSetpoint", () -> coralIntakePivot.setPowerCmd(() -> 1));
		testModeChooser.addCmd(
			"Outtake",
			() -> coralIntake.setHasCoralInSimCmd(true).andThen(coralIntake.outtakeCmd())
		);
		testModeChooser.addCmd("Score L4", () -> botCommands.scoreSequence(4));
		if (RobotBase.isSimulation()) {
			testModeChooser.addCmd(
				"Stow and simulate coral",
				() -> coralIntake.setHasCoralInSimCmd(true).andThen(botCommands.stow())
			);
		}
		testModeChooser.addCmd("Move to L3", () -> botCommands.moveTo(Setpoint.score(3)));
		testModeChooser.addCmd(
			"Wheel radius characterization",
			drivetrain::wheelRadiusCharacterization
		);
		testModeChooser.addCmd(
			"Align(repulsor)",
			() -> drivetrain.pathfindCmd(drivetrain::getDemoPose, setpointGen)
		);
		testModeChooser.addCmd(
			"Align(basic)",
			() -> drivetrain.alignCmd(drivetrain::getDemoPose)
		);
		
		SmartDashboard.putData("TestChooser", testModeChooser);
		test().onTrue(testModeChooser.selectedCommandScheduler().ignoringDisable(true));
	}
}