package frc.chargers.utils.logging;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(LaserCan.Measurement.class)
public class LaserCANMeasurementLogger extends ClassSpecificLogger<LaserCan.Measurement> {
	public LaserCANMeasurementLogger() {
		super(LaserCan.Measurement.class);
	}
	
	@Override
	protected void update(EpilogueBackend logger, LaserCan.Measurement measurement) {
		if (measurement == null) {
			logger.log("isPresent", false);
			return;
		} else {
			logger.log("isPresent", true);
		}
		logger.log("distanceMm", measurement.distance_mm);
		logger.log("hasObject", measurement.distance_mm < 0.0);
		logger.log("status", switch (measurement.status) {
			case 0 -> "Ok";
			case 1 -> "Err: Noise Issue";
			case 2 -> "Err: Weak Signal";
			case 4 -> "Err: Out of Bounds";
			case 7 -> "WrapAround";
			default -> "Unknown Status";
		});
		logger.log("roiX", measurement.roi.x);
		logger.log("roiY", measurement.roi.y);
		logger.log("roiW", measurement.roi.w);
		logger.log("roiH", measurement.roi.h);
	}
}
