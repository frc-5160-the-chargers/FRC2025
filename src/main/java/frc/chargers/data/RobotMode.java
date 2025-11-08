package frc.chargers.data;

import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.Logger;

public enum RobotMode {
    REAL, SIM, REPLAY;

    /** Fetches the current robot mode. */
    public static RobotMode get() {
        if (RobotBase.isSimulation()) {
            return Logger.hasReplaySource() ? REPLAY : SIM;
        } else {
            return REAL;
        }
    }

    /** A convenience method for checking if the current robot mode is simulation. */
    public static boolean isSim() {
        return get() == SIM;
    }
}
