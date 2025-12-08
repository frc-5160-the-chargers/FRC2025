package frc.chargers.misc;

import edu.wpi.first.wpilibj.DriverStation;

public class CurrAlliance {
    /** Returns true when the current alliance is red. */
    public static boolean red() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
    }

    /** Returns true when the current alliance is blue. */
    public static boolean blue() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Blue;
    }
}
