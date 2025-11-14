package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ChargerSubsystem extends SubsystemBase {
    /** A convenience method for fetching a relative logging key for this subsystem. */
    public String key(String path) {
        return getName() + "/" + path;
    }
}
