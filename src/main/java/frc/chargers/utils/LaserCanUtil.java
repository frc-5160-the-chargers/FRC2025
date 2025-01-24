package frc.chargers.utils;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

public class LaserCanUtil {
	private LaserCanUtil() {}
	
	public static final LaserCan.Measurement NULL_OP_MEASUREMENT =
		new LaserCan.Measurement(
			-1, -1, -1, false, -1,
			new LaserCan.RegionOfInterest(-1, -1, -1, -1)
		);
	
	@CustomLoggerFor(LaserCan.Measurement.class)
	public static class EpilogueLogger extends ClassSpecificLogger<LaserCan.Measurement> {
		public EpilogueLogger() {
			super(LaserCan.Measurement.class);
		}
		
		@Override
		protected void update(EpilogueBackend logger, LaserCan.Measurement measurement) {
			logger.log("isNull", measurement == null || measurement.status == -1);
			if (measurement == null) return;
			logger.log("distance_mm", measurement.distance_mm);
			logger.log("ambient", measurement.ambient);
			logger.log("budget_ms", measurement.budget_ms);
			logger.log("is_long", measurement.is_long);
			logger.log("status", switch (measurement.status) {
				case -1 -> "Value is Null(-1)"; // custom status
				case 0 -> "Ok";
				case 1 -> "Err: Noise Issue(1)";
				case 2 -> "Err: Weak Signal(2)";
				case 4 -> "Err: Out of Bounds(4)";
				case 7 -> "WrapAround(7)";
				default -> "Unknown Status";
			});
		}
	}
}
