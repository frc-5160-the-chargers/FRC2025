//package frc.chargers.utils;
//
//import edu.wpi.first.epilogue.EpilogueConfiguration;
//import edu.wpi.first.epilogue.logging.EpilogueBackend;
//import edu.wpi.first.epilogue.logging.FileBackend;
//import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
//import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj.DataLogManager;
//import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.button.Trigger;
//
//import java.util.HashMap;
//
//import static monologue.Monologue.GlobalLog;
//
//public class LoggingUtil {
//	public static void logToNtInNonComp(EpilogueConfiguration config) {
//		var fileOnlyBackend = new FileBackend(DataLogManager.getLog());
//		var fileAndNtBackend = EpilogueBackend.multi(
//			new NTEpilogueBackend(NetworkTableInstance.getDefault()),
//			fileOnlyBackend
//		);
//		config.backend = fileAndNtBackend;
//		DataLogManager.logNetworkTables(false);
//		new Trigger(DriverStation::isFMSAttached)
//			.onTrue(Commands.runOnce(() -> config.backend = fileOnlyBackend))
//			.onFalse(Commands.runOnce(() -> config.backend = fileAndNtBackend));
//	}
//
//	private static class CommandCache {
//		public boolean isActive = false;
//		public int numRunningInstances = 0;
//		public int numTotalRuns = 0;
//
//		public void log(String commandName) {
//			GlobalLog.log("commands/" + commandName + "/isActive", isActive);
//			GlobalLog.log("commands/" + commandName + "/num", numRunningInstances);
//			GlobalLog.log("commands/" + commandName + "/numTotalRuns", numTotalRuns);
//		}
//	}
//	private static HashMap<String>
//
//	public static void enableCommandLogging() {
//
//	}
//}
