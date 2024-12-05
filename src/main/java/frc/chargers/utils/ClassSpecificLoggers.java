package frc.chargers.utils;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.DataLogger;

/**
 * These are class-specific loggers used by epilogue internally.
 * You do not need to use these classes yourself.
 */
public class ClassSpecificLoggers {
	private ClassSpecificLoggers(){}
	
	@CustomLoggerFor(CANBus.class)
	public static class CANBusLogger extends ClassSpecificLogger<CANBus> {
		public CANBusLogger() { super(CANBus.class); }
		
		@Override
		protected void update(DataLogger dataLogger, CANBus canBus) {
			dataLogger.log("Name", canBus.getName());
			dataLogger.log("IsNetworkFD", canBus.isNetworkFD());
			var status = canBus.getStatus();
			dataLogger.log("Status/BusUtilization", status.BusUtilization);
			dataLogger.log("Status/BusOffCount", status.BusOffCount);
			dataLogger.log("Status/ReceiveErrorCount", status.REC);
			dataLogger.log("Status/TransmitErrorCount", status.TEC);
			dataLogger.log("Status/TransmitBufferFullCount", status.TxFullCount);
		}
	}
}
