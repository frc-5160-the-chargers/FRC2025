package frc.chargers.commands;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.RequiredArgsConstructor;

import java.util.function.BooleanSupplier;

public class TriggerUtil {
	/** "Binds" an alert to a trigger/boolean supplier; pushing it to the dashboard when the condition returns true. */
	public static void bind(Alert alert, BooleanSupplier condition) {
		if (condition.getAsBoolean()) alert.set(true); // covers initial condition
		new Trigger(condition)
			.onTrue(
				Commands.runOnce(() -> alert.set(true))
					.ignoringDisable(true)
					.withName("[Enable Alert] " + alert.getText())
			)
			.onFalse(
				Commands.runOnce(() -> alert.set(false))
					.ignoringDisable(true)
					.withName("[Disable Alert] " + alert.getText())
			);
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
