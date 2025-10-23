package frc.chargers.data;

import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.Logger;

public enum RobotMode {
    REAL, SIM, REPLAY;

    public static RobotMode get() {
        if (RobotBase.isSimulation()) {
            return Logger.hasReplaySource() ? REPLAY : SIM;
        } else {
            return REAL;
        }
    }
}
