package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.LogLocal;

public abstract class StandardSubsystem extends SubsystemBase implements LogLocal, AutoCloseable {
	/** Closes and frees all resources of the subsystem. Do not use this on the real robot. */
	@Override
	public void close() throws Exception {
		System.out.println("Warning: subsystem " + getName() + " closed without closing resources.");
	}
	
	protected abstract void requestStop();
	
	/** A command that stops the subsystem once. */
	public Command stopCmd() {
		return this.runOnce(this::requestStop).withName("stop command(" + getName() + ")");
	}
	
	/** A command that stops the subsystem indefinitely, until cancelled. */
	public Command idleCmd() {
		return this.run(this::requestStop).withName("idle command(" + getName() + ")");
	}
}
