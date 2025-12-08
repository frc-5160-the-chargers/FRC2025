// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.chargers.misc;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import org.littletonrobotics.junction.Logger;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Supplier;

/**
 * A Utility class for tracing code execution time. Will put info to NetworkTables under the
 * "Tracer" table. The times outputted by the {@link Tracer} are in milliseconds.
 *
 * <p>Example inside {@code Robot.java}
 *
 * <pre><code>
 * public void loopFunc() {
 *     // The robot's entire periodic loop must be traced for Tracer to work.
 *     Tracer.trace("Main", super::loopFunc);
 * }
 *
 * public void robotPeriodic() {
 *   Tracer.trace("CommandScheduler", CommandScheduler.getInstance()::run); // CommandScheduler is already traced
 *   Tracer.trace("MyVendorDep", MyVendorDep::updateAll);
 * }
 * </code></pre>
 *
 * <p>Example inside a {@code Drive Subsystem}
 *
 * <pre><code>
 * // Subsystem periodics are automatically traced
 * public void periodic() {
 *   for (var module : modules) {
 *     Tracer.trace("Module" + module.getName(), module::update);
 *   }
 *   Tracer.trace("Gyro", gyro::update);
 * }
 * </code></pre>
 */
public class Tracer {
	private static final class TraceStartData {
		private double startTime = 0.0;
		private double startGCTotalTime = 0.0;

		private void set(double startTime, double startGCTotalTime) {
			this.startTime = startTime;
			this.startGCTotalTime = startGCTotalTime;
		}
	}

	/**
	 * All of the tracers persistent state in a single object to be stored in a {@link ThreadLocal}.
	 */
	private static final class TracerState {
		// the stack of traces, every startTrace will add to this stack
		// and every endTrace will remove from this stack
		private final ArrayList<String> traceStack = new ArrayList<>();
		// ideally we only need `traceStack` but in the interest of memory optimization
		// and string concatenation speed we store the history of the stack to reuse the stack names
		private final ArrayList<String> traceStackHistory = new ArrayList<>();
		// the time of each trace, the key is the trace name, modified every endTrace
		private final HashMap<String, Double> traceTimes = new HashMap<>();
		// the start time of each trace and the gc time at the start of the trace,
		// the key is the trace name, modified every startTrace and endTrace.
		private final HashMap<String, TraceStartData> traceStartTimes = new HashMap<>();
		private final ArrayList<String> entryArray = new ArrayList<>();

		// the garbage collector beans
		private final ArrayList<GarbageCollectorMXBean> gcs =
			new ArrayList<>(ManagementFactory.getGarbageCollectorMXBeans());

        private TracerState() {}

		private String appendTraceStack(String trace) {
			traceStack.add(trace);
			StringBuilder sb = new StringBuilder();
			for (int i = 0; i < traceStack.size(); i++) {
				sb.append(traceStack.get(i));
				if (i < traceStack.size() - 1) {
					sb.append("/");
				}
			}
			String str = sb.toString();
			traceStackHistory.add(str);
			return str;
		}

		private String popTraceStack() {
			traceStack.remove(traceStack.size() - 1);
			return traceStackHistory.remove(traceStackHistory.size() - 1);
		}

		private double totalGCTime() {
			double gcTime = 0;
			for (GarbageCollectorMXBean gc : gcs) {
				gcTime += gc.getCollectionTime();
			}
			return gcTime;
		}

		private void endCycle() {
			// update times for all already existing entries
			for (var entry : entryArray) {
				// if the entry isn't found, time will null-cast to 0.0
				Double time = traceTimes.remove(entry);
				if (time == null) time = 0.0;
				Logger.recordOutput("Tracer/" + entry, time);
			}
			// log all new entries
			for (var traceTime : traceTimes.entrySet()) {
				Logger.recordOutput("Tracer/" + traceTime.getKey(), traceTime.getValue());
				entryArray.add(traceTime.getKey());
			}

            // clean up state
			traceTimes.clear();
			traceStackHistory.clear();
		}
	}

	private static final ThreadLocal<TracerState> threadLocalState =
		ThreadLocal.withInitial(TracerState::new);

	public static void disableGcLoggingForCurrentThread() {
		TracerState state = threadLocalState.get();
		state.gcs.clear();
	}

	private static void startTrace(final String name, final TracerState state) {
		if (RobotMode.get() == RobotMode.REPLAY) return;
		String stack = state.appendTraceStack(name);
		TraceStartData data = state.traceStartTimes.get(stack);
		if (data == null) {
			data = new TraceStartData();
			state.traceStartTimes.put(stack, data);
		}
		data.set(RobotController.getFPGATime() / 1000.0, state.totalGCTime());
	}

	private static void endTrace(final TracerState state) {
		if (RobotMode.get() == RobotMode.REPLAY) return;
		try {
			String stack = state.popTraceStack();
			var startData = state.traceStartTimes.get(stack);
			double gcTimeSinceStart = state.totalGCTime() - startData.startGCTotalTime;
            state.traceTimes.put(
				stack, RobotController.getFPGATime() / 1000.0 - startData.startTime - gcTimeSinceStart);
			if (state.traceStack.isEmpty()) {
				state.endCycle();
			}
		} catch (Exception e) {
			DriverStation.reportError(
				"[Tracer] An end trace was called with no opening trace " + e, true);
		}
	}

	/**
	 * Starts a trace, should be called at the beginning of a function that's not being called by user
	 * code.
	 *
	 * @param name the name of the trace, should be unique to the function.
	 */
	public static void startTrace(String name) {
		startTrace(name, threadLocalState.get());
	}

	/**
	 * Ends a trace, should only be called at the end of a function that's not being called by user
	 * code. If a {@link Tracer#startTrace(String)} is not paired with an endTrace() call
	 * there could be dropped or incorrect data.
	 */
	public static void endTrace() {
		endTrace(threadLocalState.get());
	}

	/**
	 * Runs and traces a function.
	 *
	 * @param name the name of the trace, should be unique to the function.
	 * @param runnable the function to trace.
	 * @apiNote If you want to return a value then use {@link Tracer#trace(String, Supplier)}.
	 */
	public static void trace(String name, Runnable runnable) {
		final TracerState state = threadLocalState.get();
		try {
			startTrace(name, state);
			runnable.run();
		} finally {
			endTrace(state);
		}
	}

	/**
	 * Runs and traces a function.
	 *
	 * @param name the name of the trace, should be unique to the function.
	 * @param supplier the function to trace.
	 */
	public static <T> T trace(String name, Supplier<T> supplier) {
		final TracerState state = threadLocalState.get();
		try {
			return supplier.get();
		} finally {
			endTrace(state);
		}
	}

	/**
	 * Returns a command that has it's execute() method traced.
	 *
	 * @param name the name of the trace, should be unique to the function.
	 * @param command the command to trace.
	 */
	public static Command trace(String name, Command command) {
		return new WrapperCommand(command) {
			@Override
			public void execute() {
				startTrace(name);
				super.execute();
				endTrace();
			}

			@Override
			public String getName() {
				return name;
			}
		};
	}
}