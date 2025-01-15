package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.LogLocal;

public abstract class StandardSubsystem extends SubsystemBase implements LogLocal, AutoCloseable {
	/** Closes and frees all resources of the subsystem. Do not use this on the real robot. */
	@Override
	public void close() throws Exception {
		// default implementation
		System.out.println("Warning: subsystem " + getName() + " closed without closing resources.");
	}
}
