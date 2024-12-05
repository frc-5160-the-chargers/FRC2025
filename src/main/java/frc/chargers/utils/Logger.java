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

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.MalformedURLException;
import java.net.URL;
import java.util.*;
import java.util.function.BooleanSupplier;

/**
 * A custom wrapper to DogLog that adds imperative alert logging and a default configure method.
 * Used in the same way as DogLog(and inherits the methods from it).
 */
@SuppressWarnings("unused")
public class Logger extends DogLog {
	private static final Map<String, Alert> errorAlerts = new LinkedHashMap<>();
	private static final Map<String, Alert> infoAlerts = new LinkedHashMap<>();
	// codes make the color blue
	private static final String INFO_START = "\u001B[36m(INFO) ";
	private static final String INFO_END = "\u001B[0m";
	private static final DataLogger fileOnlyBackend = new FileLogger(DataLogManager.getLog());
	private static final DataLogger fileAndNtBackend = DataLogger.multi(
		fileOnlyBackend, new NTDataLogger(NetworkTableInstance.getDefault())
	);
	
	private Logger() {}
	
	private static void disableNtLogging() {
		logInfo("Competition Started: NT(Live) logging disabled.");
		DogLog.setOptions(DogLog.getOptions().withNtPublish(false));
		Epilogue.getConfig().dataLogger = fileOnlyBackend;
	}
	
	private static void enableNtLogging() {
		logInfo("NT(Live) Logging Enabled.");
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
	
	/** Logs Radio information(stolen from advantagekit) */
	public static void logRadio() {
		RadioLogger.periodic();
	}
	
	private static class RadioLogger {
		private static final double requestPeriodSecs = 5.0;
		private static final int connectTimeout = 500;
		private static final int readTimeout = 500;
		
		private static URL statusURL;
		private static Notifier notifier;
		private static final Object lock = new Object();
		private static boolean isConnected = false;
		private static String statusJson = "";
		
		public static void periodic() {
			if (notifier == null && RobotBase.isReal()) {
				start();
			}
			
			synchronized (lock) {
				Logger.log("Radio/connected", isConnected);
				Logger.log("Radio/status", statusJson);
			}
		}
		
		private static void start() {
			// Get status URL
			int teamNumber = RobotController.getTeamNumber();
			StringBuilder statusURLBuilder = new StringBuilder();
			statusURLBuilder.append("http://10.");
			statusURLBuilder.append(teamNumber / 100);
			statusURLBuilder.append(".");
			statusURLBuilder.append(teamNumber % 100);
			statusURLBuilder.append(".1/status");
			try {
				statusURL = new URL(statusURLBuilder.toString());
			} catch (MalformedURLException e) {
				return;
			}
			
			// Launch notifier
			notifier = new Notifier(
				() -> {
					// Request status from radio
					StringBuilder response = new StringBuilder();
					try {
						HttpURLConnection connection = (HttpURLConnection) statusURL.openConnection();
						connection.setRequestMethod("GET");
						connection.setConnectTimeout(connectTimeout);
						connection.setReadTimeout(readTimeout);
						
						try (BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()))) {
							for (String line; (line = reader.readLine()) != null; ) {
								response.append(line);
							}
						}
					} catch (Exception e) {}
					
					// Update status
					String responseStr = response.toString().replaceAll("\\s+", "");
					synchronized (lock) {
						isConnected = !responseStr.isEmpty();
						statusJson = responseStr;
					}
				});
			notifier.setName("DogLog_RadioLogger");
			notifier.startPeriodic(requestPeriodSecs);
		}
	}
}
