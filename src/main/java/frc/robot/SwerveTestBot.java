package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.chargers.utils.InputStream;
import frc.chargers.utils.StatusSignalRefresher;
import frc.chargers.utils.TunableValues;
import frc.robot.subsystems.swerve.SwerveConfigurator;
import frc.robot.subsystems.swerve.SwerveDrive;
import monologue.LogLocal;
import monologue.Monologue;
import org.ironmaple.simulation.SimulatedArena;

import static monologue.Monologue.GlobalLog;

@Logged
public class SwerveTestBot extends TimedRobot implements LogLocal {
	private final SwerveDrive drivetrain = new SwerveDrive(
		SwerveConfigurator.HARDWARE_SPECS,
		SwerveConfigurator.CONTROLS_CONFIG,
		SwerveConfigurator.MODULE_TYPE,
		SwerveConfigurator.DEFAULT_MOTOR_CONFIG,
		Rotation2d::new
	);
	@NotLogged private final CommandXboxController driverController = new CommandXboxController(0);
	private final AutoFactory autoFactory = drivetrain.createAutoFactory();
	
	public SwerveTestBot() {
		// Required for ChargerTalonFX and ChargerCANcoder to work
		StatusSignalRefresher.startPeriodic(this);
		// logging setup(required)
		Epilogue.bind(this);
		Monologue.setup(this, Epilogue.getConfig());
		DataLogManager.start();
		logMetadata();
		
		addPeriodic(drivetrain::updateOdometry, 1 / SwerveConfigurator.ODOMETRY_FREQUENCY_HZ);
		// enables tuning mode
		TunableValues.setTuningMode(true);
		DriverStation.silenceJoystickConnectionWarning(true);
		SmartDashboard.putData(
			"View Connection warnings",
			Commands.runOnce(() -> DriverStation.silenceJoystickConnectionWarning(false))
		);
		
		mapTriggers();
		mapDefaultCommands();
		log("hasInitialized", true);
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
	
	@Override
	public void autonomousInit() {
		autoFactory.resetOdometry("SimplePath")
			.andThen(autoFactory.trajectoryCmd("SimplePath"))
			.schedule();
	}
	
	private void mapTriggers() {
		driverController.a()
			.whileTrue(drivetrain.runDriveMotors());
		driverController.b()
			.whileTrue(drivetrain.runTurnMotors());
		driverController.x()
			.whileTrue(drivetrain.setSteerAngles(new Rotation2d[]{
				Rotation2d.fromDegrees(-45),
				Rotation2d.fromDegrees(45),
				Rotation2d.fromDegrees(45),
				Rotation2d.fromDegrees(-45),
			}));
		driverController.y()
			.whileTrue(drivetrain.setSteerAngles(Rotation2d.kZero));
	}
	
	private void mapDefaultCommands() {
		drivetrain.setDefaultCommand(
			drivetrain.driveCmd(
				InputStream.of(driverController::getLeftY)
					.deadband(0.1, 1)
					.times(-0.5)
					.log("driverController/xOutput"),
				InputStream.of(driverController::getLeftX)
					.deadband(0.1, 1)
					.times(-0.5)
					.log("driverController/yOutput"),
				InputStream.of(driverController::getRightX)
					.deadband(0.1, 1)
					.times(-0.5)
					.log("driverController/rotationOutput"),
				false
			)
		);
	}
	
	private void logMetadata() {
		GlobalLog.logMetadata("GitDate", BuildConstants.GIT_DATE);
		GlobalLog.logMetadata("BuildDate", BuildConstants.BUILD_DATE);
		GlobalLog.logMetadata("GitBranch", BuildConstants.GIT_BRANCH);
		GlobalLog.logMetadata("GitDirty", Integer.toString(BuildConstants.DIRTY));
		GlobalLog.logMetadata("GitSHA", BuildConstants.GIT_SHA);
	}
}