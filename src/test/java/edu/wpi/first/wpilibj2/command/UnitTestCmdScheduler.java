package edu.wpi.first.wpilibj2.command;

public class UnitTestCmdScheduler {
	/** Use UnitTestCmdScheduler.get(). */
	private UnitTestCmdScheduler() {}
	
	public static CommandScheduler create() {
		return new CommandScheduler();
	}
}
