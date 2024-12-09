package frc.chargers.utils;

import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.logging.DataLogger;
import edu.wpi.first.epilogue.logging.FileLogger;
import edu.wpi.first.epilogue.logging.NTDataLogger;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A custom wrapper to DogLog that adds imperative alert logging and a default configure method.
 * Used in the same way as DogLog(and inherits the methods from it).
 */
@SuppressWarnings("unused")
public class Logger extends DogLog {
	private static final DataLogger fileOnlyBackend = new FileLogger(DataLogManager.getLog());
	private static final DataLogger fileAndNtBackend = DataLogger.multi(
		fileOnlyBackend, new NTDataLogger(NetworkTableInstance.getDefault())
	);
	
	private Logger() {}
	
	private static void disableNtLogging() {
		System.out.println("Competition Started: NT(Live) logging disabled.");
		DogLog.setOptions(DogLog.getOptions().withNtPublish(false));
		Epilogue.getConfig().dataLogger = fileOnlyBackend;
	}
	
	private static void enableNtLogging() {
		System.out.println("NT(Live) Logging Enabled.");
		DogLog.setOptions(DogLog.getOptions().withNtPublish(true));
		Epilogue.getConfig().dataLogger = fileAndNtBackend;
	}
	
	/**
	 * Starts the logger.
	 * This must be called alongside Epilogue.bind(this) in the Robot class.
	 * In addition, the robot class must have the @Logged annotation.
	 */
	public static void configureDefault() {
		// Epilogue by default logs to nt only, while DogLog defaults to file-only
		enableNtLogging();
		DogLog.setPdh(new PowerDistribution());
		
		// enables NT logging when FMS is absent
		new Trigger(DriverStation::isFMSAttached)
			.onTrue(Commands.runOnce(Logger::disableNtLogging))
			.onFalse(Commands.runOnce(Logger::enableNtLogging));
		
		// Logs when commands are running or not.
		var scheduler = CommandScheduler.getInstance();
		scheduler.onCommandInitialize(cmd -> log("RunningCommands/" + cmd.getName(), true));
		scheduler.onCommandFinish(cmd -> log("RunningCommands/" + cmd.getName(), false));
		scheduler.onCommandInterrupt(cmd -> log("RunningCommands/" + cmd.getName(), false));
		// capture alert NT data
		NetworkTableInstance.getDefault().startEntryDataLog(
			DataLogManager.getLog(),
			"SmartDashboard/Alerts",
			"SmartDashboard/Alerts"
		);
	}
}
