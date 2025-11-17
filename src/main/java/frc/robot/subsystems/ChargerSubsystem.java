package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public abstract class ChargerSubsystem extends SubsystemBase {
    public ChargerSubsystem() {
        CommandScheduler.getInstance().getActiveButtonLoop()
            .bind(() -> {
                var currCmd = getCurrentCommand();
                Logger.recordOutput(
                    key("currentCommand"),
                    currCmd == null ? "none" : currCmd.getName()
                );
            });
    }

    /** A convenience method for fetching a relative logging key for this subsystem. */
    public String key(String path) {
        return getName() + "/" + path;
    }
}
