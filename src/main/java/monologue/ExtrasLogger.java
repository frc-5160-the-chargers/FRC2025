package monologue;

import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.hal.PowerJNI;
import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import org.jetbrains.annotations.Nullable;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.time.Duration;

/** Logs "extra" information. */
public class ExtrasLogger {
	private static boolean enabled = false;
	private static final CANStatus status = new CANStatus();
	private static PowerDistribution pdh;
	private static EpilogueBackend logger = null;
	
	private static final RadioLogUtil radioLogUtil = new RadioLogUtil();
	private static final GenericPublisher ntRadioStatus =
		NetworkTableInstance.getDefault().getTopic("RadioStatus/StatusJSON").genericPublish("json");
	private static final StringLogEntry dataLogRadioStatus =
		new StringLogEntry(DataLogManager.getLog(), "RadioStatus/StatusJSON", "", "json");
	private static final GenericPublisher ntRadioConn =
		NetworkTableInstance.getDefault().getTopic("RadioStatus/Connected").genericPublish("json");
	private static final BooleanLogEntry dataLogRadioConn =
		new BooleanLogEntry(DataLogManager.getLog(), "RadioStatus/Connected");
	
	/** Starts extras logging. Called once in Robot constructor. */
	public static void start(TimedRobot robot, @Nullable PowerDistribution pdh) {
		if (ExtrasLogger.enabled) {
			RuntimeLog.warn("ExtrasLogger.start has already been called, further calls will do nothing");
			return;
		}
		if (!Monologue.hasBeenSetup()) {
			RuntimeLog.warn("ExtrasLogger started before monologue was setup - nothing will be logged");
			return;
		}
		ExtrasLogger.enabled = true;
		ExtrasLogger.pdh = pdh;
		ExtrasLogger.logger = Monologue.config.backend.getNested("SystemStats");
		robot.addPeriodic(() -> {
			try {
				ExtrasLogger.logSystem();
				ExtrasLogger.logCan();
				ExtrasLogger.logPdh();
			} catch (Exception e) {
				DriverStation.reportError("ExtrasLogger err: " + e, true);
			}
		}, 0.02);
		new Notifier(ExtrasLogger::logRadio).startPeriodic(5.160); // go chargers!
	}
	
	private static void logSystem() {
		if (logger == null) return;
		logger.log("SerialNumber", HALUtil.getSerialNumber());
		logger.log("Comments", HALUtil.getComments());
		logger.log("SystemActive", HAL.getSystemActive());
		logger.log("BrownedOut", HAL.getBrownedOut());
		logger.log("RSLState", HAL.getRSLState());
		logger.log("SystemTimeValid", HAL.getSystemTimeValid());
		logger.log("BrownoutVoltage", PowerJNI.getBrownoutVoltage());
		logger.log("CPUTempCelcius", PowerJNI.getCPUTemp());
		
		logger.log("BatteryVoltage", PowerJNI.getVinVoltage());
		logger.log("BatteryCurrent", PowerJNI.getVinCurrent());
		
		logger.log("3v3Rail/Voltage", PowerJNI.getUserVoltage3V3());
		logger.log("3v3Rail/Current", PowerJNI.getUserCurrent3V3());
		logger.log("3v3Rail/Active", PowerJNI.getUserActive3V3());
		logger.log("3v3Rail/CurrentFaults", PowerJNI.getUserCurrentFaults3V3());
		
		logger.log("5vRail/Voltage", PowerJNI.getUserVoltage5V());
		logger.log("5vRail/Current", PowerJNI.getUserCurrent5V());
		logger.log("5vRail/Active", PowerJNI.getUserActive5V());
		logger.log("5vRail/CurrentFaults", PowerJNI.getUserCurrentFaults5V());
		
		logger.log("6vRail/Voltage", PowerJNI.getUserVoltage6V());
		logger.log("6vRail/Current", PowerJNI.getUserCurrent6V());
		logger.log("6vRail/Active", PowerJNI.getUserActive6V());
		logger.log("6vRail/CurrentFaults", PowerJNI.getUserCurrentFaults6V());
	}
	
	private static void logCan() {
		if (logger == null) return;
		CANJNI.getCANStatus(status);
		logger.log("CANBus/Utilization", status.percentBusUtilization);
		logger.log("CANBus/OffCount", status.busOffCount);
		logger.log("CANBus/TxFullCount", status.txFullCount);
		logger.log("CANBus/ReceiveErrorCount", status.receiveErrorCount);
		logger.log("CANBus/TransmitErrorCount", status.transmitErrorCount);
	}
	
	private static void logPdh() {
		if (pdh == null || logger == null) return;
		logger.log("PowerDistribution/Temperature", pdh.getTemperature());
		logger.log("PowerDistribution/Voltage", pdh.getVoltage());
		logger.log("PowerDistribution/ChannelCurrent", pdh.getAllCurrents());
		logger.log("PowerDistribution/TotalCurrent", pdh.getTotalCurrent());
		logger.log("PowerDistribution/TotalPower", pdh.getTotalPower());
		logger.log("PowerDistribution/TotalEnergy", pdh.getTotalEnergy());
		logger.log("PowerDistribution/ChannelCount", pdh.getNumChannels());
	}
	
	private static void logRadio() {
		radioLogUtil.refresh();
		ntRadioConn.setBoolean(radioLogUtil.radioLogResult.isConnected);
		dataLogRadioConn.append(radioLogUtil.radioLogResult.isConnected);
		ntRadioStatus.setString(radioLogUtil.radioLogResult.statusJson);
		dataLogRadioStatus.append(radioLogUtil.radioLogResult.statusJson);
	}
	
	private static class RadioLogResult {
		public String statusJson = "";
		public boolean isConnected = false;
	}
	
	private static class RadioLogUtil {
		private static final Duration REQUEST_TIMEOUT_DURATION = Duration.ofSeconds(1);
		
		private static URI getRadioStatusEndpoint() {
			var teamNumber = RobotController.getTeamNumber();
			
			return URI.create("http://10." + teamNumber / 100 + "." + teamNumber % 100 + ".1/status");
		}
		
		public final RadioLogResult radioLogResult = new RadioLogResult();
		
		private final HttpClient httpClient = HttpClient.newHttpClient();
		private final HttpRequest request =
			HttpRequest.newBuilder()
				.uri(getRadioStatusEndpoint())
				.timeout(REQUEST_TIMEOUT_DURATION)
				.build();
		
		/** Get the latest radio status and update the {@link this#radioLogResult} object. */
		public void refresh() {
			try {
				var response = httpClient.send(request, HttpResponse.BodyHandlers.ofString());
				
				radioLogResult.isConnected = true;
				radioLogResult.statusJson = response.body();
			} catch (Exception e) {
				radioLogResult.isConnected = false;
				radioLogResult.statusJson = "";
			}
		}
	}
}
