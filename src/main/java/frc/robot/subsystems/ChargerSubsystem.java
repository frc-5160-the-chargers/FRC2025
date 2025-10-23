package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ChargerSubsystem extends SubsystemBase {
    public String key(String path) {
        return getName() + "/" + path;
    }
}
