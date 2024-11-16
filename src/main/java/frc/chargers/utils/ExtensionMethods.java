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
import java.util.function.Supplier;

public class ExtensionMethods {
    private ExtensionMethods() {}
	
	// Generic extensions
	/** Runs the supplier, returning its result. */
	public static <T> T run(Supplier<T> toRun){ return toRun.get(); }

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
	public static Trigger doubleClicked(Trigger base) {
		return doubleClicked(base, 0.4);
	}
	
	/**
	 * Creates a new trigger that returns true when the receiver is double-clicked.
	 * Usage: <code>controller.x().doubleClicked().onTrue(command)</code>
	 */
	public static Trigger doubleClicked(Trigger base, double maxLengthSeconds) {
		var tracker = new DoublePressTracker(base, maxLengthSeconds);
		return new Trigger(tracker::get);
	}

	// Motor Extension Methods
	/** Configures a spark motor with default ResetMode and PersistMode options. */
	public static REVLibError resetAndConfigure(SparkBase motor, SparkBaseConfig config) {
		return motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
	}
	
	// Alliance flip util (will be removed soon)
	static Pose2d flip(Pose2d pose, SymmetryStrategy strategy) {
		return new Pose2d(
			flip(pose.getTranslation(), strategy),
			flip(pose.getRotation(), strategy)
		);
	}
	
	static Rotation2d flip(Rotation2d rotation, SymmetryStrategy strategy) {
		return switch (strategy) {
			case VERTICAL -> new Rotation2d(-rotation.getCos(), rotation.getSin());
			case ROTATIONAL -> new Rotation2d(-rotation.getCos(), -rotation.getSin());
			case HORIZONTAL -> new Rotation2d(rotation.getCos(), -rotation.getSin());
		};
	}
	
	static Translation2d flip(Translation2d translation, SymmetryStrategy strategy) {
		return new Translation2d(
			strategy.flipX(translation.getX()),
			strategy.flipY(translation.getY())
		);
	}
}
