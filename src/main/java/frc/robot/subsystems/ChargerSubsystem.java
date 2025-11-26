package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.chargers.misc.Tracer;
import org.littletonrobotics.junction.Logger;

public abstract class ChargerSubsystem extends SubsystemBase {
    /** A convenience method for fetching a relative logging key for this subsystem. */
    public String key(String path) {
        return getName() + "/" + path;
    }

    /**
     * If you override this method instead of periodic(),
     * extra logging will be added by default.
     */
    public void loggedPeriodic() {}

    @Override
    public void periodic() {
        Tracer.startTrace(getName() + " Periodic");
        loggedPeriodic();
        var currCmd = getCurrentCommand();
        Logger.recordOutput(
            key("CurrentCommand"),
            currCmd == null ? "none" : currCmd.getName()
        );
        Tracer.endTrace();
    }
}
