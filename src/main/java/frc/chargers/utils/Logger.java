package frc.chargers.utils;

import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.logging.DataLogger;
import edu.wpi.first.epilogue.logging.FileLogger;
import edu.wpi.first.epilogue.logging.NTDataLogger;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.*;
import java.util.function.BooleanSupplier;

/**
 * A custom wrapper to DogLog that adds imperative alert logging and a default configure method.
 * Used in the same way as DogLog(and inherits the methods from it).
 */
@SuppressWarnings("unused")
public class Logger extends DogLog {
	private static final Map<String, Alert> errorAlerts = new HashMap<>();
	private static final Map<String, Alert> infoAlerts = new HashMap<>();
	// codes make the color blue
	private static final String INFO_START = "\u001B[36m(INFO) ";
	private static final String INFO_END = "\u001B[0m";
	private static final DataLogger fileOnlyBackend = new FileLogger(DataLogManager.getLog());
	private static final DataLogger fileAndNtBackend = DataLogger.multi(
		fileOnlyBackend, new NTDataLogger(NetworkTableInstance.getDefault())
	);
	
	private Logger() {}
	
	private static void disableNtLogging() {
		Logger.logInfo("Competition Started: NT(Live) logging disabled.");
		DogLog.setOptions(DogLog.getOptions().withNtPublish(false));
		Epilogue.configure(config -> config.dataLogger = fileOnlyBackend);
	}
	
	private static void enableNtLogging() {
		Logger.logInfo("NT(Live) Logging Enabled.");
		DogLog.setOptions(DogLog.getOptions().withNtPublish(true));
		Epilogue.configure(config -> config.dataLogger = fileAndNtBackend);
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
	}
	
	/** Logs a fault to datalog and displays an alert. */
	public static void logFault(String faultName) {
		DogLog.logFault(faultName);
		errorAlerts
			.computeIfAbsent(faultName, (k) -> {
				DriverStation.reportError(faultName, false);
				return new Alert(faultName, AlertType.kError);
			})
			.set(true);
	}
	
	/** Logs a fault to datalog and displays an alert. */
	public static void logFault(Enum<?> fault) { Logger.logFault(fault.name()); }
	
	/**
	 * Logs a fault whenever the condition function returns true.
	 * Should be called only once.
	 */
	public static void logFaultWhen(BooleanSupplier condition, String faultName) {
		new Trigger(condition).onTrue(Commands.runOnce(() -> Logger.logFault(faultName)));
	}
	
	/**
	 * Logs a fault whenever the condition function returns true.
	 * Should be called only once.
	 */
	public static void logFaultWhen(BooleanSupplier condition, Enum<?> fault) {
		Logger.logFaultWhen(condition, fault.toString());
	}
	
	/** Logs useful information to the console(in blue) and displays an info alert. */
	public static void logInfo(String info) {
		infoAlerts
			.computeIfAbsent(info, k -> {
				System.out.println(INFO_START + info + INFO_END);
				return new Alert(info, AlertType.kInfo);
			})
			.set(true);
	}
}
