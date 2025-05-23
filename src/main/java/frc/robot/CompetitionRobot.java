package frc.robot;

import choreo.auto.AutoChooser;
import com.ctre.phoenix6.SignalLogger;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.chargers.utils.LaserCanUtil;
import frc.chargers.utils.Tracer;
import frc.chargers.utils.data.StatusSignalRefresher;
import frc.chargers.utils.data.TunableValues;
import frc.chargers.utils.data.TunableValues.TunableBool;
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
import frc.robot.subsystems.AlgaeKicker;
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

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.autonomous;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.test;
import static frc.chargers.utils.TriggerUtil.bind;
import static frc.chargers.utils.TriggerUtil.doubleClicked;
import static frc.chargers.utils.UtilMethods.waitThenRun;
import static frc.robot.constants.OtherConstants.*;
import static monologue.Monologue.GlobalLog;

/*
Reminder: There is a rio 2 only GC config on lines 39-44 of build.gradle.
Comment it out if we are running a rio 1.
 */
@Logged
public class CompetitionRobot extends TimedRobot implements LogLocal {
	/** State that has to be shared across subsystems. */
	public static class SharedState {
		public BooleanSupplier atL1Range;
		public BooleanSupplier hasCoralDelayed;
		public DoubleSupplier elevatorSpeed;
		public DoubleSupplier headingTimestampSecs;
		public Supplier<Rotation2d> headingSupplier;
	}
	
	/* Subsystems/Components */
	private final TunableBool fieldRelativeToggle = new TunableBool("swerveDrive/fieldRelative", true);
	private final SharedState sharedState = new SharedState();
	private final GyroWrapper gyroWrapper = new GyroWrapper();
	private final SwerveDrive drivetrain = new SwerveDrive(
		SwerveConfigurator.HARDWARE_SPECS,
		SwerveConfigurator.CONTROLS_CONFIG,
		SwerveConfigurator.MODULE_TYPE,
		SwerveConfigurator.DEFAULT_MOTOR_CONFIG,
		gyroWrapper::yaw
	);
	private final Elevator elevator = new Elevator(sharedState);
	private final CoralIntake coralIntake = new CoralIntake(sharedState);
	private final CoralIntakePivot coralIntakePivot = new CoralIntakePivot(sharedState);
	private final AlgaeKicker algaeKicker = new AlgaeKicker();
	private final AprilTagVision vision = new AprilTagVision(sharedState);
	
	/* Generic constants/utility classes */
	private final RobotVisualization visualizer =
		new RobotVisualization(drivetrain, coralIntake, coralIntakePivot, elevator);
	@NotLogged private final SwerveSetpointGenerator setpointGen =
		drivetrain.createSetpointGenerator(SwerveConfigurator.BODY_MOI, SwerveConfigurator.DRIVE_CURRENT_LIMIT);
	private final TargetPoses targetPoses =
		new TargetPoses(REEF_SCORE_OFFSET, SOURCE_OFFSET, SwerveConfigurator.HARDWARE_SPECS);
	
	/* Commands */
	@NotLogged private final RobotCommands botCommands =
		new RobotCommands(drivetrain, coralIntake, coralIntakePivot, elevator);
	@NotLogged private final AutoCommands autoCommands =
		new AutoCommands(botCommands, drivetrain.createAutoFactory(), coralIntake, drivetrain, targetPoses, setpointGen);
	
	/* Auto choosers */
	@NotLogged private final AutoChooser autoChooser = new AutoChooser();
	@NotLogged private final AutoChooser testModeChooser = new AutoChooser();
	
	/* Controllers/Driver input */
	private final DriverController driver = new DriverController();
	private final OperatorController operator = new OperatorController();
	
	public CompetitionRobot() {
		// Initializes shared state
		sharedState.headingSupplier = () -> drivetrain.bestPose().getRotation();
		sharedState.headingTimestampSecs = gyroWrapper::getLastHeadingTimestamp;
		sharedState.atL1Range = () -> elevator.heightMeters() < 0.15;
		sharedState.hasCoralDelayed = coralIntake.hasCoral.debounce(0.7);
		sharedState.elevatorSpeed = elevator::velocityMPS;
		
		// calls runTcp() and setups jni
		LaserCanUtil.setup(true);
		// enables tuning mode
		TunableValues.setTuningMode(true);
		
		// logging setup(required)
		Epilogue.bind(this);
		Monologue.setup(this, Epilogue.getConfig());
		DataLogManager.start();
		GlobalLog.enableCommandLogging();
		logMetadata();
		URCL.start();
		SignalLogger.stop();
		ExtrasLogger.start(this, null);
		
		mapTriggers();
		mapDefaultCommands();
		mapAutoModes();
		mapTestCommands();
		
		addPeriodic(() -> {
			gyroWrapper.refreshYaw();
			drivetrain.updateOdometry();
		}, 1 / SwerveConfigurator.ODOMETRY_FREQUENCY_HZ);
		// Vision setup - there are 2 overloads for addVisionData
		vision.setGlobalEstimateConsumer(drivetrain::addVisionData);
		vision.setSimPoseSupplier(drivetrain::bestPose);
		DriverStation.silenceJoystickConnectionWarning(true);
		
		if (RobotBase.isSimulation()) {
			SimulatedArena.getInstance().placeGamePiecesOnField();
			drivetrain.resetPose(new Pose2d(5, 7, Rotation2d.kZero));
		} else {
			waitThenRun(2, () -> drivetrain.resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(120))));
		}
	}
	
	@Override
	public void robotPeriodic() {
		// Tracer.trace(...) runs the method while recording loop times
		Tracer.trace("cmd scheduler", CommandScheduler.getInstance()::run);
		Tracer.trace("mech visualizer", visualizer::periodic);
		Tracer.trace("CAN signal refresh", StatusSignalRefresher::periodic);
		Tracer.trace("vision", vision::periodic);
		if (RobotBase.isSimulation()) {
			Tracer.trace("maple sim", SimulatedArena.getInstance()::simulationPeriodic);
		}
	}
	
	@Override
	public void loopFunc() { // WARNING: Do not modify this method.
		Tracer.trace("main loop", super::loopFunc);
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
		
		new Trigger(() -> !fieldRelativeToggle.get())
			.whileTrue(
				drivetrain.driveCmd(driver.forwardOutput, driver.strafeOutput, driver.rotationOutput, false)
			);
		
		driver.L1()
			.whileTrue(
				drivetrain.pathfindCmd(() -> targetPoses.closestReefPose(ReefSide.LEFT, drivetrain.poseEstimate()), setpointGen)
			);
		driver.R1()
			.whileTrue(
				drivetrain.pathfindCmd(() -> targetPoses.closestReefPose(ReefSide.RIGHT, drivetrain.poseEstimate()), setpointGen)
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
					drivetrain.resetPose(new Pose2d(currPose.getX(), currPose.getY(), Rotation2d.kZero));
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
			.whileTrue(botCommands.sourceIntake());
		operator.leftBumper()
			.whileTrue(botCommands.stow());
		
		// backward
		operator.leftTrigger().whileTrue(algaeKicker.setPowerCmd(() -> 0.07));
		// forward
		operator.rightTrigger().whileTrue(algaeKicker.setPowerCmd(() -> -0.07));
		
		operator.a()
			.whileTrue(botCommands.moveTo(Setpoint.score(1)));
		operator.b()
			.whileTrue(botCommands.moveTo(Setpoint.score(2)));
		operator.y()
			.whileTrue(botCommands.moveTo(Setpoint.score(3)));
		operator.x()
			.whileTrue(botCommands.moveTo(Setpoint.score(4)));
		
		doubleClicked(operator.start())
			.onTrue(Commands.runOnce(() -> drivetrain.resetPose(drivetrain.getDemoPose())).ignoringDisable(true));
	}
	
	private void mapDefaultCommands() {
		drivetrain.setDefaultCommand(
			drivetrain.driveCmd(driver.forwardOutput, driver.strafeOutput, driver.rotationOutput, false)
		);
		elevator.setDefaultCommand(elevator.setPowerCmd(operator.manualElevatorInput));
		coralIntake.setDefaultCommand(coralIntake.idleCmd());
		coralIntakePivot.setDefaultCommand(coralIntakePivot.setPowerCmd(operator.manualPivotInput));
		algaeKicker.setDefaultCommand(algaeKicker.idleCmd());
	}
	
	private void logMetadata() {
		GlobalLog.logMetadata("GitDate", BuildConstants.GIT_DATE);
		GlobalLog.logMetadata("BuildDate", BuildConstants.BUILD_DATE);
		GlobalLog.logMetadata("GitBranch", BuildConstants.GIT_BRANCH);
		GlobalLog.logMetadata("GitDirty", Integer.toString(BuildConstants.DIRTY));
		GlobalLog.logMetadata("GitSHA", BuildConstants.GIT_SHA);
	}
	
	private void mapAutoModes() {
		autoChooser.addCmd("Taxi", autoCommands::taxi);
		autoChooser.addCmd("3x L4 Right", autoCommands::tripleL4South);
		autoChooser.addCmd("4x L1 Right", autoCommands::quadL1South);
		autoChooser.addCmd("L4 L1 L1 Right", autoCommands::l4L1L1South);
		autoChooser.addCmd("L4 L4 L1 Right", autoCommands::l4L4L1South);
		autoChooser.addCmd("One Piece L4", () -> autoCommands.onePieceL4(false));
		autoChooser.addCmd("One Piece L4(mirrored)", () -> autoCommands.onePieceL4(true));
		autoChooser.addCmd("One Piece L4(center)", autoCommands::onePieceL4Center);
		autoChooser.addCmd("One Piece L1", autoCommands::onePieceL1);
		autoChooser.addCmd("(TEST ONLY) figure eight", autoCommands::figureEight);
		autoChooser.addCmd("(TEST ONLY) multi piece", autoCommands::multiPieceTest);
		autoChooser.addCmd("Stupid fing reset pose test", autoCommands::resetOdoTest);
		autoChooser.select("Stupid fing reset pose test");
		
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
			() -> drivetrain.pathfindCmd(() -> targetPoses.reefBlue[8], setpointGen)
		);
		testModeChooser.addCmd(
			"Align(basic)",
			() -> drivetrain.alignCmd(drivetrain::getDemoPose)
		);
		testModeChooser.addCmd(
			"Reset Pose",
			() -> Commands.runOnce(() -> drivetrain.resetPose(drivetrain.getDemoPose()))
		);
		testModeChooser.addCmd(
			"Set Steer angles",
			() -> drivetrain.setSteerAngles(Rotation2d.k180deg.plus(Rotation2d.kCW_90deg))
		);
		testModeChooser.addCmd(
			"HelloThere",
			() -> drivetrain.pathfindCmd(() -> targetPoses.closestReefPose(ReefSide.LEFT, drivetrain.poseEstimate()), setpointGen)
		);
		testModeChooser.addCmd(
			"DriveVoltages",
			drivetrain::runDriveMotors
		);
		testModeChooser.addCmd(
			"AlgaeKickerTest",
			algaeKicker::setDemoVoltageCmd
		);
		
		SmartDashboard.putData("TestChooser", testModeChooser);
		test().onTrue(testModeChooser.selectedCommandScheduler().ignoringDisable(true));
	}
}