package frc.chargers.commands;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.RequiredArgsConstructor;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;

public class TriggerUtil {
	/** "Binds" an alert to a trigger/boolean supplier; pushing it to the dashboard when the condition returns true. */
	public static void bind(Alert alert, BooleanSupplier condition) {
		CommandScheduler.getInstance().getActiveButtonLoop()
			.bind(() -> alert.set(condition.getAsBoolean()));
	}

	/**
	 * Creates a new trigger that stays true for only a single loop cycle,
	 * before going back down to false.
	 */
	public static Trigger risingEdge(Trigger receiver) {
		var previous = new AtomicBoolean(receiver.getAsBoolean());
		return new Trigger(() -> {
			boolean current = receiver.getAsBoolean();
			boolean risingEdge = current && !previous.get();
			previous.set(current);
			return risingEdge;
		});
	}
	
	/**
	 * Creates a new trigger that returns true when the receiver is double-clicked.
	 * <p>
	 * Usage: <code>doubleClicked(controller.x()).onTrue(command)</code> <br />
	 * With @ExtensionMethod: <code>controller.x().doubleClicked().onTrue(command)</code>
	 * </p>
	 */
	public static Trigger doubleClicked(Trigger receiver) {
		return doubleClicked(receiver, 0.4);
	}
	
	/**
	 * Creates a new trigger that returns true when the receiver is double-clicked.
	 */
	public static Trigger doubleClicked(Trigger receiver, double maxLengthSeconds) {
		var tracker = new DoublePressTracker(receiver, maxLengthSeconds);
		return new Trigger(tracker::get);
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
