package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.epilogue.logging.FileBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.chargers.utils.AutoChooser;
import frc.chargers.utils.InputStream;
import frc.chargers.utils.UtilExtensionMethods;
import frc.robot.commands.AutoCommands;
import frc.robot.subsystems.swerve.SwerveConfigurator;
import frc.robot.subsystems.swerve.SwerveDrive;
import lombok.experimental.ExtensionMethod;
import monologue.LogLocal;
import monologue.Monologue;
import org.ironmaple.simulation.SimulatedArena;

import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.autonomous;

@ExtensionMethod(UtilExtensionMethods.class)
public class DriverPracticeSim extends TimedRobot implements LogLocal {
	@Logged public final SwerveDrive drivetrainOne = createSimBot(
		new Pose2d(5.0, 7.0, Rotation2d.kZero)
	);
	private final AutoChooser autoChooser = new AutoChooser();

	private int currControllerId = 0;
	private SwerveDrive createSimBot(Pose2d initialPose) {
		var drivetrain = new SwerveDrive(SwerveConfigurator.defaultConfig());
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
		// logging config; do not remove
		Epilogue.bind(this);
		Monologue.setup(this, Epilogue.getConfig());
		Epilogue.getConfig().errorHandler = ErrorHandler.crashOnError();
		Epilogue.getConfig().backend =
			EpilogueBackend.multi(
				new NTEpilogueBackend(NetworkTableInstance.getDefault()),
				new FileBackend(DataLogManager.getLog())
			);

		SimulatedArena.getInstance().placeGamePiecesOnField();
		mapAutoModes();
		DriverStation.silenceJoystickConnectionWarning(true);
	}

	private void mapAutoModes() {
		var autoCommands = new AutoCommands(drivetrainOne.createAutoFactory());
		autoChooser.addCmd("FourPiece", autoCommands::fourPiece);
		autoChooser.addCmd("Characterize", drivetrainOne::characterizeFeedforwardCmd);
		autonomous().whileTrue(autoChooser.selectedCommandScheduler().withName("AutoCmdScheduler"));
		SmartDashboard.putData("AutoChooser", autoChooser);
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		if (RobotBase.isSimulation()) {
			SimulatedArena.getInstance().simulationPeriodic();
			// occasionally, maplesim can output NaN battery voltages
			log(
				"simNotePoses",
				SimulatedArena.getInstance()
					.getGamePiecesByType("Note")
					.toArray(new Pose3d[]{})
			);
		}
	}
}
