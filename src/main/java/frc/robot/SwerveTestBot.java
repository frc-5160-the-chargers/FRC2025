//package frc.robot;
//
//import choreo.auto.AutoFactory;
//import edu.wpi.first.epilogue.Epilogue;
//import edu.wpi.first.epilogue.Logged;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.wpilibj.DataLogManager;
//import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.RobotBase;
//import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import edu.wpi.first.wpilibj2.command.Commands;
//import frc.chargers.utils.Tracer;
//import frc.chargers.utils.data.InputStream;
//import frc.chargers.utils.data.StatusSignalRefresher;
//import frc.chargers.utils.data.TunableValues;
//import frc.robot.components.controllers.DriverController;
//import frc.robot.subsystems.swerve.SwerveConfigurator;
//import frc.robot.subsystems.swerve.SwerveDrive;
//import monologue.LogLocal;
//import monologue.Monologue;
//import org.ironmaple.simulation.SimulatedArena;
//
///** Robot for testing swerve and running Swerve routines. */
//@Logged
//public class SwerveTestBot extends TimedRobot implements LogLocal {
//	private final SwerveDrive drivetrain = new SwerveDrive(
//		SwerveConfigurator.HARDWARE_SPECS,
//		SwerveConfigurator.CONTROLS_CONFIG,
//		SwerveConfigurator.MODULE_TYPE,
//		SwerveConfigurator.DEFAULT_MOTOR_CONFIG,
//		Rotation2d::new
//	);
//	private final DriverController driver = new DriverController();
//	private final AutoFactory autoFactory = drivetrain.createAutoFactory();
//
//	public SwerveTestBot() {
//		// Required for ChargerTalonFX and ChargerCANcoder to work
//		StatusSignalRefresher.startPeriodic(this);
//		// logging setup(required)
//		Epilogue.bind(this);
//		Monologue.setup(this, Epilogue.getConfig());
//		DataLogManager.start();
//
//		addPeriodic(drivetrain::updateOdometry, 1 / SwerveConfigurator.ODOMETRY_FREQUENCY_HZ);
//		// enables tuning mode
//		TunableValues.setTuningMode(true);
//		DriverStation.silenceJoystickConnectionWarning(true);
//		SmartDashboard.putData(
//			"View Connection warnings",
//			Commands.runOnce(() -> DriverStation.silenceJoystickConnectionWarning(false))
//		);
//
//		mapTriggers();
//		mapDefaultCommands();
//		log("hasInitialized", true);
//	}
//
//	@Override
//	public void robotPeriodic() {
//		// All of this code is required
//		var startTime = System.nanoTime();
//		Tracer.trace("CmdScheduler", CommandScheduler.getInstance()::run);
//		if (RobotBase.isSimulation()) {
//			SimulatedArena.getInstance().simulationPeriodic();
//			log("simulatedCoralPositions", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
//			log("simulatedAlgaePositions", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
//		}
//		log("loopRuntime", (System.nanoTime() - startTime) / 1e6);
//	}
//
//	@Override
//	public void autonomousInit() {
//		autoFactory.resetOdometry("SimplePath")
//			.andThen(autoFactory.trajectoryCmd("SimplePath"))
//			.schedule();
//	}
//
//	private void mapTriggers() {
//		driver.triangle()
//			.whileTrue(drivetrain.runDriveMotors());
//		driver.square()
//			.whileTrue(drivetrain.runTurnMotors());
//		driver.circle()
//			.whileTrue(drivetrain.setSteerAngles(
//				Rotation2d.fromDegrees(-45),
//				Rotation2d.fromDegrees(45),
//				Rotation2d.fromDegrees(45),
//				Rotation2d.fromDegrees(-45)
//			));
//		driver.cross()
//			.whileTrue(drivetrain.setSteerAngles(Rotation2d.kZero));
//	}
//
//	private void mapDefaultCommands() {
//		drivetrain.setDefaultCommand(
//			drivetrain.driveCmd(
//				InputStream.of(driver::getLeftY)
//					.deadband(0.1, 1)
//					.times(-0.5)
//					.log("driverController/xOutput"),
//				InputStream.of(driver::getLeftX)
//					.deadband(0.1, 1)
//					.times(-0.5)
//					.log("driverController/yOutput"),
//				InputStream.of(driver::getRightX)
//					.deadband(0.1, 1)
//					.times(-0.5)
//					.log("driverController/rotationOutput"),
//				false
//			)
//		);
//	}
//}