package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.EpilogueConfiguration;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frc.robot.subsystems.swerve.SwerveConfigurator;
import frc.robot.subsystems.swerve.SwerveDrive;
import monologue.LogLocal;
import monologue.Monologue;

public class MoreTesting extends TimedRobot implements LogLocal {
	private SwerveDrive drivetrain = new SwerveDrive(SwerveConfigurator.defaultConfig());
	
	@Logged double y = 0.0;
	
	public MoreTesting() {
		LiveWindow.disableAllTelemetry();
		Monologue.setup(this, new EpilogueConfiguration());
		log("Hi", 2.0);
		new PIDController(0.5, 0.0, 0.0);
	}
}
