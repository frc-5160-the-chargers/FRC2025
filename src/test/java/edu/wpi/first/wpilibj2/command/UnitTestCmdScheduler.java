package edu.wpi.first.wpilibj2.command;

/**
 * A workaround for the inability to create new command schedulers in unit tests.
 */
public class UnitTestCmdScheduler {
	/** Use UnitTestCmdScheduler.create(). */
	private UnitTestCmdScheduler() {}
	
	public static CommandScheduler create() {
		return new CommandScheduler();
	}
}
