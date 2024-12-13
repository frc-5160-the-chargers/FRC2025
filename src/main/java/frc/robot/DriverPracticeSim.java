package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule;
import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.chargers.utils.SwerveSetpointGenerator;
import frc.chargers.utils.InputStream;
import frc.chargers.utils.UtilExtensionMethods;
import frc.chargers.utils.UtilMethods;
import frc.robot.subsystems.swerve.SwerveConfigurator;
import frc.robot.subsystems.swerve.SwerveDrive;
import lombok.experimental.ExtensionMethod;
import org.ironmaple.simulation.SimulatedArena;
import org.photonvision.simulation.VisionSystemSim;

import static edu.wpi.first.epilogue.Logged.Strategy.OPT_IN;
import static edu.wpi.first.math.geometry.AllianceSymmetry.SymmetryStrategy;

@Logged(strategy = OPT_IN)
@ExtensionMethod(UtilExtensionMethods.class)
public class DriverPracticeSim extends TimedRobot {
	@Logged private final SwerveDrive drivetrainOne = createSimBot(
		"drivetrainOne",
		new Pose2d(5.0, 7.0, Rotation2d.kZero)
	);
	@Logged private final SwerveDrive drivetrainTwo = createSimBot(
		"drivetrainTwo",
		new Pose2d(5.0, 7.0, Rotation2d.kZero).flip(SymmetryStrategy.ROTATIONAL)
	);
	
	private final VisionSystemSim photonSimBase = new VisionSystemSim(null);
	
	private int currControllerId = 0;
	private SwerveDrive createSimBot(String name, Pose2d initialPose) {
		var drivetrain = new SwerveDrive(name, SwerveConfigurator.getDefaultConfig());
		drivetrain.resetPose(initialPose);
		
		var controller = new CommandXboxController(currControllerId);
		currControllerId++;
		
		drivetrain.setDefaultCommand(
			drivetrain.teleopDriveCmd(
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
		// logging config
		Epilogue.bind(this);
		UtilMethods.configureDefaultLogging();
	}
	
	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		SimulatedArena.getInstance().simulationPeriodic();
	}
}
