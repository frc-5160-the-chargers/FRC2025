package frc.robot.components;

import au.grapplerobotics.CanBridge;
import au.grapplerobotics.GrappleJNI;
import au.grapplerobotics.LaserCan;
import org.littletonrobotics.junction.AutoLog;

/**
 * A LaserCan with akit interop.
 * Note that the forceLoad() and runTcp() statements should be included
 * regardless of whether you're using akit or not.
 */
public class LoggedLaserCan {
    static {
        GrappleJNI.forceLoad();
        CanBridge.runTCP();
    }

    public enum LaserCanStatus {
        SEEN, NOTHING_SEEN, NOISE_ISSUE, WEAK_SIGNAL, OUT_OF_BOUNDS, WRAPAROUND
    }
    
    @AutoLog
    public static class LaserCanData {
        public double distance_mm = -1;
        public int ambient = -1;
        public int budgetMs = -1;
        public boolean isLong = false;
        public LaserCanStatus status = LaserCanStatus.NOTHING_SEEN;
    }
    
    private final LaserCan laserCan;

    public LoggedLaserCan(int canId) {
        this.laserCan = new LaserCan(canId);
    }

    public void refreshData(LaserCanDataAutoLogged inputs) {
        var measurement = laserCan.getMeasurement();
        if (measurement == null || measurement.status == -1) {
            inputs.status = LaserCanStatus.NOTHING_SEEN;
            inputs.distance_mm = -1;
            inputs.ambient = -1;
            inputs.budgetMs = -1;
        } else {
            inputs.status = switch (measurement.status) {
                case 0 -> LaserCanStatus.SEEN;
                case 1 -> LaserCanStatus.NOISE_ISSUE;
                case 2 -> LaserCanStatus.WEAK_SIGNAL;
                case 4 -> LaserCanStatus.OUT_OF_BOUNDS;
                case 7 -> LaserCanStatus.WRAPAROUND;
                default -> LaserCanStatus.NOTHING_SEEN;
            };
            inputs.distance_mm = measurement.distance_mm;
            inputs.ambient = measurement.ambient;
            inputs.budgetMs = measurement.budget_ms;
            inputs.isLong = measurement.is_long;
        }
    }
}
