
package edu.wpi.first.wpilibj2.command;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.simulation.SimHooks;

import static edu.wpi.first.units.Units.Seconds;

/**
 * A utility class for scheduling and running commands.
 */
public class CommandTestingUtil {
	public static final Time TICK_RATE = Seconds.of(0.02);
	
	/**
	 * Creates a new command scheduler.
	 * Use this instead of CommandScheduler.getInstance() in unit tests.
	 */
	public static CommandScheduler newCommandScheduler() {
		return new CommandScheduler();
	}
	
	/**
	 * Runs CommandScheduler and updates timer repeatedly
	 * to fast-forward subsystems and run commands.
	 *
	 * @param ticks The number of times CommandScheduler is run
	 */
	public static void runTicks(CommandScheduler scheduler, int ticks) {
		for (int i = 0; i < ticks; i++) {
			scheduler.run();
			SimHooks.stepTiming(TICK_RATE.in(Seconds));
		}
	}
	
	/**
	 * Fasts forward in time by running CommandScheduler and updating timer.
	 */
	public static void fastForward(CommandScheduler scheduler, Time time) {
		runTicks(scheduler, timeToTicks(time));
	}
	
	/**
	 * Fetches the number of ticks that occur in the time interval.
	 */
	public static int timeToTicks(Time time) {
		return (int) (time.in(Seconds) / TICK_RATE.in(Seconds));
	}
	
	/**
	 * Schedules and runs a command.
	 */
	public static void runUntilComplete(CommandScheduler scheduler, Command command, Time timeout) {
		scheduler.schedule(command);
		double time = 0;
		runTicks(scheduler, 1);
		while (command.isScheduled() && time < timeout.in(Seconds)) {
			runTicks(scheduler, 1);
			time += 0.02;
		}
	}
}
