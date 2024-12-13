package frc.chargers.utils.betatesting;

import edu.wpi.first.epilogue.logging.DataLogger;

/**
 * A class that can be manually logged
 */
public interface ManualLogged {
	default DataLogger logger() {
		return ManualLogProcessor.loggerOf(this);
	}
}
