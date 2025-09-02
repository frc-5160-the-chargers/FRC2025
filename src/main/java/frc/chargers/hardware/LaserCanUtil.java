package frc.chargers.hardware;

import au.grapplerobotics.CanBridge;
import au.grapplerobotics.GrappleJNI;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

public class LaserCanUtil {
	private LaserCanUtil() {}

	/** Workaround method for LaserCAN not working. */
	public static void setup(boolean useTcp) {
		GrappleJNI.forceLoad();
		if (useTcp) CanBridge.runTCP();
	}

	/** A null-op measurement, used in place of null values retreived from a laser can. */
	public static final LaserCan.Measurement NULL_OP_MEASUREMENT =
		new LaserCan.Measurement(
			-1, -1, -1, false, -1,
			new LaserCan.RegionOfInterest(-1, -1, -1, -1)
		);

	/** A custom epilogue logger for lasercan measurements. */
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
				case 0 -> "Ok(0)";
				case 1 -> "Err: Noise Issue(1)";
				case 2 -> "Err: Weak Signal(2)";
				case 4 -> "Err: Out of Bounds(4)";
				case 7 -> "WrapAround(7)";
				default -> "Unknown Status";
			});
		}
	}
}
