package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.chargers.utils.LiveData;
import frc.robot.commands.RobotCommands;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CoralIntakePivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.SwerveConfigurator;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.vision.AprilTagVision;
import monologue.LogLocal;
import monologue.Monologue;
import org.ironmaple.simulation.SimulatedArena;

import static frc.chargers.utils.UtilMethods.configureDefaultLogging;
import static frc.chargers.utils.UtilMethods.scheduleSequentially;

@Logged
public class Robot extends TimedRobot implements LogLocal {
	private final SwerveDrive drivetrain = new SwerveDrive(SwerveConfigurator.DEFAULT_CONFIG);
	private final AlgaeIntake algaeIntake = new AlgaeIntake();
	private final Climber climber = new Climber();
	private final CoralIntake coralIntake = new CoralIntake();
	private final CoralIntakePivot coralIntakePivot = new CoralIntakePivot();
	private final Elevator elevator = new Elevator();
	private final AprilTagVision vision = new AprilTagVision(this);
	
	@NotLogged private final CommandPS5Controller driverController = new CommandPS5Controller(0);
	@NotLogged private final CommandXboxController manualOperatorController = new CommandXboxController(1);
	
	// logging doesnt really work for sendables
	@NotLogged private final AutoChooser autoChooser = new AutoChooser();
	
	private final RobotCommands cmdFactory = new RobotCommands(drivetrain, coralIntake, coralIntakePivot, elevator);
	
	public Robot() {
		// logging setup
		Epilogue.bind(this);
		Monologue.setup(this, Epilogue.getConfig());
		configureDefaultLogging(Epilogue.getConfig());
		// enables tuning mode
		LiveData.setTuningMode(true);
		
		mapTriggers();
		mapDefaultCommands();
		mapAutoModes();
		vision.setVisionConsumer(drivetrain);
	}
	
	private void mapTriggers() {
		// TODO
	}
	
	private void mapDefaultCommands() {
		drivetrain.setDefaultCommand(
			drivetrain.driveCmd(
				driverController::getLeftX,
				driverController::getLeftY,
				driverController::getRightX,
				true
			)
		);
		// TODO - Set other default commands here
	}
	
	private void mapAutoModes() {
		// TODO
		autoChooser.addCmd("Testing", () -> scheduleSequentially(
			Commands.print("A"),
			Commands.print("B"),
			Commands.print("C")
		));
		SmartDashboard.putData("AutoChooser", autoChooser);
		RobotModeTriggers.autonomous().onTrue(autoChooser.selectedCommandScheduler());
	}
	
	@Override
	public void robotPeriodic() {
		var startTime = System.nanoTime();
		CommandScheduler.getInstance().run();
		if (RobotBase.isSimulation()) {
			SimulatedArena.getInstance().simulationPeriodic();
//			log("simulatedCoralPositions", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
//			log("simulatedAlgaePositions", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
		}
		log("loopRuntime", (System.nanoTime() - startTime) / 1e6);
	}
}
