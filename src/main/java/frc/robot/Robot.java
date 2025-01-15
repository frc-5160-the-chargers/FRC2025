package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.chargers.utils.LiveData;
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

import static frc.chargers.utils.UtilMethods.configureDefaultLogging;

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
	
	@NotLogged private final AutoChooser autoChooser = new AutoChooser();
	
	private String formatCommandData(Command cmd) {
		return cmd.getName() + "; Requirements: " + cmd.getRequirements();
	}
	
	public Robot() {
		// logging setup
		Epilogue.bind(this);
		Monologue.setup(this, Epilogue.getConfig());
		configureDefaultLogging(Epilogue.getConfig());
		// enables tuning mode
		LiveData.setTuningMode(true);
		
		var scheduler = CommandScheduler.getInstance();
		scheduler.onCommandInitialize(cmd -> SmartDashboard.putBoolean(cmd.getName(), true));
		scheduler.onCommandInterrupt(cmd -> SmartDashboard.putBoolean(cmd.getName(), false));
		scheduler.onCommandFinish(cmd -> SmartDashboard.putBoolean(cmd.getName(), false));
		
		mapTriggers();
		mapDefaultCommands();
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
		// TODO - Change these to appropriate commands
		algaeIntake.setDefaultCommand(algaeIntake.idleCmd());
		coralIntake.setDefaultCommand(coralIntake.idleCmd());
		climber.setDefaultCommand(Commands.idle(climber));
		coralIntakePivot.setDefaultCommand(coralIntakePivot.idleCmd());
		elevator.setDefaultCommand(Commands.idle(elevator));
	}
}
