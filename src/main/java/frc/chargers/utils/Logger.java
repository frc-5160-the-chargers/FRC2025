package frc.chargers.utils;

import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.logging.FileLogger;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.HashMap;
import java.util.Map;

public class Logger extends DogLog {
	/**
	 * Starts the logger.
	 * This must be called alongside Epilogue.bind(this) in the Robot class.
	 * In addition, the robot class must have the @Logged annotation.
	 */
	public static void startDefault() {
		DogLog.setOptions(DogLog.getOptions().withNtPublish(true).withCaptureDs(true));
		DogLog.setPdh(new PowerDistribution());
		
		var defaultEpilogueLogger = Epilogue.getConfig().dataLogger;
		var fileOnlyEpilogueLogger = new FileLogger(DataLogManager.getLog());
		new Trigger(DriverStation::isFMSAttached)
			.onTrue(
				Commands.runOnce(() -> {
					System.out.println("Competition Started: NT logging has been disabled.");
					DogLog.setOptions(DogLog.getOptions().withNtPublish(false));
					Epilogue.configure(config -> config.dataLogger = fileOnlyEpilogueLogger);
				})
			)
			.onFalse(
				Commands.runOnce(() -> {
					System.out.println("NT Logging has been re-enabled.");
					DogLog.setOptions(DogLog.getOptions().withNtPublish(true));
					Epilogue.configure(config -> config.dataLogger = defaultEpilogueLogger);
				})
			);
	}
	
	private static final Map<String, Alert> errorAlerts = new HashMap<>();
	private static final Map<String, Alert> infoAlerts = new HashMap<>();
	// codes make the color blue
	private static final String INFO_START = "\u001B[36m(INFO) ";
	private static final String ERROR_START = "\u001B[31m(ERROR) ";
	private static final String MSG_END = "\u001B[0m";
	
	public static void logFault(String faultName) {
		DogLog.logFault("(ERROR) " + faultName);
		errorAlerts
			.computeIfAbsent(faultName, k -> {
				System.out.println(ERROR_START + faultName + MSG_END);
				return new Alert(faultName, AlertType.kError);
			})
			.set(true);
	}
	
	public static void logFault(Enum<?> fault) { Logger.logFault(fault.name()); }
	
	public static void logInfo(String info) {
		infoAlerts
			.computeIfAbsent(info, k -> {
				System.out.println(INFO_START + info + MSG_END);
				return new Alert(info, AlertType.kInfo);
			})
			.set(true);
	}
}
