package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.LogLocal;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public abstract class StandardSubsystem extends SubsystemBase implements LogLocal, AutoCloseable {
	/** Closes and frees all resources of the subsystem. Do not use this on the real robot. */
	@Override
	public void close() throws Exception {
		System.out.println("Warning: subsystem " + getName() + " closed without closing resources.");
	}
	
	protected abstract void requestStop();
	
	/** A command that stops the subsystem once. */
	public Command stopCmd() {
		return this.runOnce(this::requestStop).withName("StopCmd(" + getName() + ")");
	}
	
	/** A command that stops the subsystem indefinitely, until cancelled. */
	public Command idleCmd() {
		return this.run(this::requestStop).withName("IdleCmd(" + getName() + ")");
	}
	
	public static Command idleAllCmd(StandardSubsystem... subsystems) {
		var idleCommands = new ArrayList<Command>();
		for (var s : subsystems) { idleCommands.add(s.idleCmd()); }
		return Commands.parallel(idleCommands.toArray(new Command[0]))
			       .withName("IdleAllCmd" + nameList(subsystems));
	}
	
	public static Command stopAllCmd(StandardSubsystem... subsystems) {
		var stopCommands = new ArrayList<Command>();
		for (var s : subsystems) { stopCommands.add(s.stopCmd()); }
		return Commands.parallel(stopCommands.toArray(new Command[0]))
			       .withName("StopAllCmd" + nameList(subsystems));
	}
	
	private static List<String> nameList(StandardSubsystem... subsystems) {
		return Arrays.stream(subsystems).map(StandardSubsystem::getName).toList();
	}
}
