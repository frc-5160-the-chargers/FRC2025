package frc.chargers.utils;

import com.pathplanner.lib.config.PIDConstants;
import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Epilogue;
//import edu.wpi.first.epilogue.logging.EpilogueBackend;
//import edu.wpi.first.epilogue.logging.FileBackend;
//import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.RequiredArgsConstructor;

import java.util.function.Supplier;

import static edu.wpi.first.wpilibj.Alert.AlertType.kInfo;

@SuppressWarnings("unused")
public class UtilMethods {
	private UtilMethods() {}
	private static final double EPSILON = 1E-9;
	
//	private static final EpilogueBackend fileOnlyBackend = new FileBackend(DataLogManager.getLog());
//	private static final EpilogueBackend fileAndNtBackend = EpilogueBackend.multi(
//		fileOnlyBackend, new NTEpilogueBackend(NetworkTableInstance.getDefault())
//	);
	private static boolean loggingConfigured = false;
	private static final Alert ntLoggingDisabled =
		new Alert("NT(Live) Logging has been disabled; FMS is attached", kInfo);
	
	/**
	 * Checks if 2 doubles are equal; correcting for floating point error.
	 * Usage: <code>equivalent(2.0, 3.0 - 1.0)</code>
	 */
	public static boolean equivalent(double a, double b) {
		return Math.abs(a - b) <= EPSILON;
	}
	
	/**
	 * Gets a trigger that returns true once when the value changes.
	 */
	public static Trigger hasChangedEvent(Supplier<?> supplier) {
		var hasChangedHandler = new HasChangedHandler(supplier);
		return new Trigger(hasChangedHandler::compute);
	}
	
	/**
	 * Creates a new trigger that returns true when the receiver is double-clicked.
	 * Usage: <code>doubleClicked(controller.x()).onTrue(command)</code>
	 * This method can also be converted to an extension method via
	 * lombok's @ExtensionMethod({Triggers.class})
	 */
	public static Trigger doubleClicked(Trigger receiver) {
		return doubleClicked(receiver, 0.4);
	}
	
	/**
	 * Creates a new trigger that returns true when the receiver is double-clicked.
	 * Usage: <code>doubleClicked(controller.x(), 0.5).onTrue(command)</code>
	 * This method can also be converted to an extension method via
	 * lombok's @ExtensionMethod({Triggers.class})
	 */
	public static Trigger doubleClicked(Trigger receiver, double maxLengthSeconds) {
		var tracker = new DoublePressTracker(receiver, maxLengthSeconds);
		return new Trigger(tracker::get);
	}
	
	/** Creates a PID controller from PID constants. */
	public static PIDController pidControllerFrom(PIDConstants constants) {
		return new PIDController(constants.kP, constants.kI, constants.kD);
	}
	
	/** Runs the toRun method immediately after the time has elapsed. */
	public static void waitThenRun(Time time, Runnable toRun) {
		Commands.waitTime(time)
			.andThen(Commands.runOnce(toRun))
			.ignoringDisable(true)
			.schedule();
	}
	
	/**
	 * Runs the logging function once after logging is configured.
	 * Must be used in constructors to log values.
	 */
	public static void logInInit(Runnable logFn) {
		Commands.waitUntil(() -> loggingConfigured)
			.andThen(logFn)
			.ignoringDisable(true)
			.schedule();
	}
	
	/**
	 * Sets the default DogLog/epilogue logging config.
	 * This must be called alongside Epilogue.bind(this) in the Robot class.
	 * In addition, the robot class must have the @Logged annotation.
	 */
	public static void configureDefaultLogging() {
		if (loggingConfigured) return;
		loggingConfigured = true;
		
		// Epilogue by default logs to nt only, while DogLog defaults to file-only
		enableNtLogging();
		DogLog.setPdh(new PowerDistribution());
		
		// enables NT logging when FMS is absent
		new Trigger(DriverStation::isFMSAttached)
			.onTrue(Commands.runOnce(UtilMethods::disableNtLogging))
			.onFalse(Commands.runOnce(UtilMethods::enableNtLogging));
		
		// Logs when commands are running or not.
		var scheduler = CommandScheduler.getInstance();
		scheduler.onCommandInitialize(cmd -> DogLog.log("runningCommands/" + cmd.getName(), true));
		scheduler.onCommandFinish(cmd -> DogLog.log("runningCommands/" + cmd.getName(), false));
		scheduler.onCommandInterrupt(cmd -> DogLog.log("runningCommands/" + cmd.getName(), false));
		// capture alert NT data
		NetworkTableInstance.getDefault().startEntryDataLog(
			DataLogManager.getLog(),
			"SmartDashboard/Alerts",
			"SmartDashboard/Alerts"
		);
	}
	
	private static void disableNtLogging() {
		ntLoggingDisabled.set(true);
		DogLog.setOptions(DogLog.getOptions().withNtPublish(false));
//		Epilogue.getConfig().backend = fileOnlyBackend;
	}
	
	private static void enableNtLogging() {
		ntLoggingDisabled.set(false);
		DogLog.setOptions(DogLog.getOptions().withNtPublish(true));
//		Epilogue.getConfig().backend = fileAndNtBackend;
	}
	
	// Credits: 6328
	
	@RequiredArgsConstructor
	private static class HasChangedHandler {
		private final Supplier<?> supplier;
		private Object value = null;
		
		public boolean compute() {
			var current = supplier.get();
			if (value != current) {
				value = current;
				return value != null;
			} else {
				return false;
			}
		}
	}
	
	@RequiredArgsConstructor
	private static class DoublePressTracker {
		private final Trigger trigger;
		private final double maxLengthSecs;
		
		private final Timer resetTimer = new Timer();
		private DoublePressState state = DoublePressState.IDLE;
		
		public boolean get() {
			boolean pressed = trigger.getAsBoolean();
			switch (state) {
				case IDLE:
					if (pressed) {
						state = DoublePressState.FIRST_PRESS;
						resetTimer.reset();
						resetTimer.start();
					}
					break;
				case FIRST_PRESS:
					if (!pressed) {
						if (resetTimer.hasElapsed(maxLengthSecs)) {
							reset();
						} else {
							state = DoublePressState.FIRST_RELEASE;
						}
					}
					break;
				case FIRST_RELEASE:
					if (pressed) {
						state = DoublePressState.SECOND_PRESS;
					} else if (resetTimer.hasElapsed(maxLengthSecs)) {
						reset();
					}
					break;
				case SECOND_PRESS:
					if (!pressed) {
						reset();
					}
			}
			return state == DoublePressState.SECOND_PRESS;
		}
		
		private void reset() {
			state = DoublePressState.IDLE;
			resetTimer.stop();
		}
	}
	
	private enum DoublePressState {
		IDLE,
		FIRST_PRESS,
		FIRST_RELEASE,
		SECOND_PRESS
	}
}
