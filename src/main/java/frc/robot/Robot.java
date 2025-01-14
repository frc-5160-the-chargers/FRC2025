package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.TimedRobot;
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
	private final SwerveDrive drivetrainOne = new SwerveDrive(SwerveConfigurator.DEFAULT_CONFIG);
	private final AlgaeIntake algaeIntake = new AlgaeIntake();
	private final Climber climber = new Climber();
	private final CoralIntake coralIntake = new CoralIntake();
	private final CoralIntakePivot coralIntakePivot = new CoralIntakePivot();
	private final Elevator elevator = new Elevator();
	private final AprilTagVision vision = new AprilTagVision(this);
	
	private final CommandPS5Controller driverController = new CommandPS5Controller(0);
	private final CommandXboxController manualOperatorController = new CommandXboxController(1);
	
	
	private final AutoChooser autoChooser = new AutoChooser();
	
	public Robot() {
		// logging setup
		Epilogue.bind(this);
		Monologue.setup(this, Epilogue.getConfig());
		configureDefaultLogging(Epilogue.getConfig());
		// enables tuning mode
		LiveData.setTuningMode(true);
		
		
	}
}
