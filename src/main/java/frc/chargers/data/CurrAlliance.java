package frc.chargers.data;

import edu.wpi.first.wpilibj.DriverStation;

public class CurrAlliance {
    public static boolean red() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
    }

    public static boolean blue() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Blue;
    }
}
