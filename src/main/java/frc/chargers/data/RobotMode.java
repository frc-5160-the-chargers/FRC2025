package frc.chargers.data;

import edu.wpi.first.wpilibj.RobotBase;

import java.util.Objects;

public enum RobotMode {
    REAL, SIM, REPLAY;

    // Running "./gradlew simulateJava -Preplay" will enable this(see build.gradle).
    private static final boolean isReplay =
        Objects.equals(System.getProperty("replayMode"), "enabled");

    /** Fetches the current robot mode. */
    public static RobotMode get() {
        if (!RobotBase.isSimulation()) return REAL;
        return isReplay ? REPLAY : SIM;
    }

    /** A convenience method for checking if the current robot mode is simulation. */
    public static boolean isSim() {
        return get() == SIM;
    }
}
