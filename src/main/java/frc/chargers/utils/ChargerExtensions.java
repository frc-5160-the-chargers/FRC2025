package frc.chargers.utils;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.geometry.AllianceSymmetry.SymmetryStrategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Consumer;

/**
 * This is a group of extension methods, built to be used by lombok.
 * <p>
 * Extension methods are methods that are instance methods of a class
 * that are not located within that class.
 *
 * <pre> {@code
 * // required for extension methods to work
 * @ExtensionMethod(ChargerExtensions.class)
 * public class Something {
 *     public Something() {
 *         // with lombok
 *         sparkMax.resetAndConfigure(config);
 *         // without lombok
 *         ChargerExtensions.resetAndConfigure(sparkMax, config);
 *     }
 * }
 * } </pre>
 * If you put your own static method here, the very first parameter will act as the
 * extension receiver.
 */
public class ChargerExtensions {
    private ChargerExtensions() {}
	
	// Generic extensions
	
	/**
	 * Runs toRun with the receiver, then returns the receiver.
	 * Usage: <code>someValue.also(it -> it.instanceMethod());</code>
	 */
	public static <T> T also(T receiver, Consumer<T> toRun) {
		toRun.accept(receiver);
		return receiver;
	}
	
	// Trigger util
	/**
	 * Creates a new trigger that returns true when the receiver is double-clicked.
	 * Usage: <code>controller.x().doubleClicked().onTrue(command)</code>
	 */
	public static Trigger doubleClicked(Trigger receiver) {
		return doubleClicked(receiver, 0.4);
	}
	
	/**
	 * Creates a new trigger that returns true when the receiver is double-clicked.
	 * Usage: <code>controller.x().doubleClicked().onTrue(command)</code>
	 */
	public static Trigger doubleClicked(Trigger receiver, double maxLengthSeconds) {
		var tracker = new DoublePressTracker(receiver, maxLengthSeconds);
		return new Trigger(tracker::get);
	}

	// Motor Extension Methods
	/** Configures a spark motor with default ResetMode and PersistMode options. */
	public static REVLibError resetAndConfigure(SparkBase receiver, SparkBaseConfig config) {
		return receiver.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
	}
	
	// Alliance flip util (will be removed soon)
	static Pose2d flip(Pose2d receiver, SymmetryStrategy strategy) {
		return new Pose2d(
			flip(receiver.getTranslation(), strategy),
			flip(receiver.getRotation(), strategy)
		);
	}
	
	static Rotation2d flip(Rotation2d receiver, SymmetryStrategy strategy) {
		return switch (strategy) {
			case VERTICAL -> new Rotation2d(-receiver.getCos(), receiver.getSin());
			case ROTATIONAL -> new Rotation2d(-receiver.getCos(), -receiver.getSin());
			case HORIZONTAL -> new Rotation2d(receiver.getCos(), -receiver.getSin());
		};
	}
	
	static Translation2d flip(Translation2d receiver, SymmetryStrategy strategy) {
		return new Translation2d(
			strategy.flipX(receiver.getX()),
			strategy.flipY(receiver.getY())
		);
	}
}
