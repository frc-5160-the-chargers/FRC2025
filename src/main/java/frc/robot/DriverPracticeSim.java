package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.chargers.utils.InferLogPath;
import frc.chargers.utils.InputStream;
import frc.chargers.utils.Logger;
import frc.chargers.utils.UtilExtensionMethods;
import frc.robot.subsystems.swerve.SwerveConfigurator;
import frc.robot.subsystems.swerve.SwerveDrive;
import lombok.experimental.ExtensionMethod;
import org.ironmaple.simulation.SimulatedArena;

import static edu.wpi.first.epilogue.Logged.Strategy.OPT_IN;
import static edu.wpi.first.math.geometry.AllianceSymmetry.SymmetryStrategy;

@Logged(strategy = OPT_IN)
@ExtensionMethod(UtilExtensionMethods.class)
public class DriverPracticeSim extends TimedRobot {
	@Logged private final SwerveDrive drivetrainOne;
	@Logged private final SwerveDrive drivetrainTwo;
	
	private int currControllerId = 0;
	private SwerveDrive createSimBot(Pose2d initialPose) {
		var drivetrain = new SwerveDrive(SwerveConfigurator.getDefaultConfig());
		drivetrain.resetPose(initialPose);
		drivetrain.setField2dEnabled(false);
		
		var controller = new CommandXboxController(currControllerId);
		currControllerId++;
		
		drivetrain.setDefaultCommand(
			drivetrain.teleopDriveCmd(
				InputStream.of(controller::getLeftY)
					.negate()
					.log("driverController/xOutput"),
				InputStream.of(controller::getLeftX)
					.negate()
					.log("driverController/yOutput"),
				InputStream.of(controller::getRightX)
					.negate()
					.log("driverController/zOutput"),
				true
			)
		);
		return drivetrain;
	}
	
	public DriverPracticeSim() {
		Epilogue.bind(this);
		Logger.configureDefault();
		InferLogPath.Parser.enable(this);
		
		drivetrainOne = createSimBot(
			new Pose2d(5.0, 7.0, Rotation2d.kZero)
		);
		drivetrainTwo = createSimBot(
			new Pose2d(5.0, 7.0, Rotation2d.kZero).flip(SymmetryStrategy.ROTATIONAL)
		);
	}
	
	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		SimulatedArena.getInstance().simulationPeriodic();
	}
}
