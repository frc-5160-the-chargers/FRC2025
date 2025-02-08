package monologue;

import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;

class RuntimeLog {
  static EpilogueBackend logger = new NTEpilogueBackend(NetworkTableInstance.getDefault());
  
  static void info(String message) {
    logger.log("MonologueSetup", message);
  }
  
  static void warn(String warning) {
    String message = "[Monologue] (WARNING) " + warning;
    new Alert("Monologue Alerts", message, Alert.AlertType.kWarning).set(true);
    DriverStationJNI.sendError(false, 1, false, message, "", "", true);
  }
}
