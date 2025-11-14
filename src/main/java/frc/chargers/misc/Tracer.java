// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.chargers.misc;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import org.littletonrobotics.junction.Logger;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
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
		private double m_startTime;
		private double m_startGCTotalTime;

		private void set(double startTime, double startGCTotalTime) {
			this.m_startTime = startTime;
			this.m_startGCTotalTime = startGCTotalTime;
		}
	}

	/**
	 * All of the tracers persistent state in a single object to be stored in a {@link ThreadLocal}.
	 */
	@SuppressWarnings("PMD.RedundantFieldInitializer")
	private static final class TracerState {

		/**
		 * The stack of traces, every startTrace will add to this stack and every endTrace will remove
		 * from this stack.
		 */
		private final ArrayList<String> m_traceStack = new ArrayList<>();

		/**
		 * Ideally we only need `traceStack` but in the interest of memory optimization and string
		 * concatenation speed we store the history of the stack to reuse the stack names.
		 */
		private final ArrayList<String> m_traceStackHistory = new ArrayList<>();

		/** The time of each trace, the key is the trace name, modified every endTrace. */
		private final HashMap<String, Double> m_traceTimes = new HashMap<>();

		/**
		 * The start time of each trace and the gc time at the start of the trace, the key is the trace
		 * name, modified every startTrace and endTrace.
		 */
		private final HashMap<String, TraceStartData> m_traceStartTimes = new HashMap<>();

		private final HashSet<String> m_keys = new HashSet<>();

		/** If the cycle is poisoned, it will warn the user and not publish any data. */
		boolean m_cyclePoisoned = false;

		/** If the tracer is disabled, it will not publish any data or do any string manipulation. */
		boolean m_disabled = false;

		/**
		 * If the tracer should be disabled next cycle and every cycle after that until this flag is set
		 * to false. Disabling is done this way to prevent disabling/enabling.
		 */
		boolean m_disableNextCycle = false;

		/**
		 * Stack size is used to keep track of stack size even when disabled, calling `EndCycle` is
		 * important when disabled or not to update the disabled state in a safe manner.
		 */
		int m_stackSize = 0;

		// the garbage collector beans
		private final ArrayList<GarbageCollectorMXBean> m_gcs =
			new ArrayList<>(ManagementFactory.getGarbageCollectorMXBeans());
		private final String name;

        private TracerState(String name) {
			this.name = "Tracer/" + (name == null ? "" : name);
		}

		private String appendTraceStack(String trace) {
			m_stackSize++;
			if (m_disabled) {
				return "";
			}
			m_traceStack.add(trace);
			StringBuilder sb = new StringBuilder();
			for (int i = 0; i < m_traceStack.size(); i++) {
				sb.append(m_traceStack.get(i));
				if (i < m_traceStack.size() - 1) {
					sb.append('/');
				}
			}
			String str = sb.toString();
			m_traceStackHistory.add(str);
			return str;
		}

		private String popTraceStack() {
			m_stackSize = Math.max(0, m_stackSize - 1);
			if (m_disabled) {
				return "";
			}
			if (m_traceStack.isEmpty() || m_traceStackHistory.isEmpty() || m_cyclePoisoned) {
				m_cyclePoisoned = true;
				return "";
			}
			m_traceStack.remove(m_traceStack.size() - 1);
			return m_traceStackHistory.remove(m_traceStackHistory.size() - 1);
		}

		private double totalGCTime() {
			double gcTime = 0;
			for (GarbageCollectorMXBean gc : m_gcs) {
				gcTime += gc.getCollectionTime();
			}
			return gcTime;
		}

		private void endCycle() {
			if (m_disabled != m_disableNextCycle || m_cyclePoisoned) {
				// Gives publishers empty times,
				// reporting no data is better than bad data
				for (String key : m_keys) {
					Logger.recordOutput("Tracer/" + key, 0.0);
				}
				return;
			} else if (!m_disabled) {
				// update times for all already existing publishers
				for (String key: m_keys) {
					Double time = m_traceTimes.remove(key);
					if (time == null) {
						time = 0.0;
					}
					Logger.recordOutput(name + key, time);
				}
				// create publishers for all new entries
				for (var traceTime : m_traceTimes.entrySet()) {
					Logger.recordOutput(name + traceTime.getKey(), traceTime.getValue());
					m_keys.add(traceTime.getKey());
				}
            }

			// clean up state
			m_traceTimes.clear();
			m_traceStackHistory.clear();

			m_disabled = m_disableNextCycle;
		}
	}

	private static final ThreadLocal<TracerState> threadLocalState =
		ThreadLocal.withInitial(() -> new TracerState(null));

	private static void startTraceInner(final String name, final TracerState state) {
		String stack = state.appendTraceStack(name);
		if (state.m_disabled) {
			return;
		}
        state.m_traceStartTimes
			.computeIfAbsent(stack, k -> new TraceStartData())
			.set(RobotController.getFPGATime() * 1e-3, state.totalGCTime());
	}

	private static void endTraceInner(final TracerState state) {
		String stack = state.popTraceStack();
		if (!state.m_disabled) {
			if (stack.isEmpty()) {
				DriverStation.reportError(
					"[Tracer] Stack is empty,"
						+ "this means that there are more endTrace calls than startTrace calls",
					true);
				return;
			}
			var startData = state.m_traceStartTimes.get(stack);
			double gcTimeSinceStart = state.totalGCTime() - startData.m_startGCTotalTime;
            state.m_traceTimes.put(
				stack, RobotController.getFPGATime() * 1e-3 - startData.m_startTime - gcTimeSinceStart);
		}
		if (state.m_traceStack.isEmpty()) {
			state.endCycle();
		}
	}

	/**
	 * Starts a trace, should be called at the beginning of a function that's not being called by user
	 * code.
	 *
	 * @param name the name of the trace, should be unique to the function.
	 */
	public static void startTrace(String name) {
		startTraceInner(name, threadLocalState.get());
	}

	/**
	 * Ends a trace, should only be called at the end of a function that's not being called by user
	 * code. If a {@link Tracer#startTrace(String)} is not paired with a {@link Tracer#endTrace()}
	 * there could be dropped or incorrect data.
	 */
	public static void endTrace() {
		endTraceInner(threadLocalState.get());
	}

	/**
	 * Disables garbage collection logging for the current thread. This can help performance in some
	 * cases.
	 */
	public static void disableGcLoggingForCurrentThread() {
		TracerState state = threadLocalState.get();
		state.m_gcs.clear();
	}


	/**
	 * Disables any tracing for the current thread. This will cause all {@link #startTrace(String)},
	 * {@link #endTrace()} and {@link #trace(String, Runnable)} to do nothing.
	 *
	 * <p>Being disabled prevents the {@link Tracer} from publishing any values to NetworkTables. This
	 * will cause all values to appear as if they're frozen at the value they were at when this
	 * function was called.
	 */
	public static void disableTracingForCurrentThread() {
		final TracerState state = threadLocalState.get();
		state.m_disableNextCycle = true;
	}

	/**
	 * Enables any tracing for the current thread. This will cause all {@link #startTrace(String)},
	 * {@link #endTrace()} and {@link #trace(String, Runnable)} to work as normal.
	 */
	public static void enableTracingForCurrentThread() {
		final TracerState state = threadLocalState.get();
		state.m_disableNextCycle = false;
		if (state.m_stackSize == 0) {
			state.m_disabled = false;
		}
	}

	/**
	 * Traces a function, should be used in place of {@link #startTrace(String)} and {@link
	 * #endTrace()} for functions called by user code like {@code CommandScheduler.run()} and other
	 * expensive functions.
	 *
	 * @param name the name of the trace, should be unique to the function.
	 * @param runnable the function to trace.
	 */
	public static void trace(String name, Runnable runnable) {
		final TracerState state = threadLocalState.get();
		startTraceInner(name, state);
		try {
			runnable.run();
		} finally {
			endTraceInner(state);
		}
	}

	/**
	 * Traces a function, should be used in place of {@link #startTrace(String)} and {@link
	 * #endTrace()} for functions called by user code.
	 *
	 * @param <R> the return type of the function.
	 * @param name the name of the trace, should be unique to the function.
	 * @param supplier the function to trace.
	 * @return the return value of the function.
	 */
	public static <R> R trace(String name, Supplier<R> supplier) {
		final TracerState state = threadLocalState.get();
		startTraceInner(name, state);
		try {
			return supplier.get();
		} finally {
			endTraceInner(state);
		}
	}

	/** This function is only to be used in tests and is package private to prevent misuse. */
	static void resetForTest() {
		threadLocalState.remove();
	}
}