package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.LogLocal;

import java.util.ArrayList;

public abstract class StandardSubsystem extends SubsystemBase implements LogLocal, AutoCloseable {
	/** Closes and frees all resources of the subsystem. Do not use this on the real robot. */
	@Override
	public void close() throws Exception {
		System.out.println("Warning: subsystem " + getName() + " closed without closing resources.");
	}
	
	public abstract Command stopCmd();
	
	public static Command stopAllCmd(StandardSubsystem... subsystems) {
		var stopCommands = new ArrayList<Command>();
		for (var s : subsystems) { stopCommands.add(s.stopCmd()); }
		return Commands.parallel(stopCommands.toArray(new Command[0]));
	}
}
