package frc.chargers.utils;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.AllianceSymmetry.SymmetryStrategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.Consumer;

/**
 * This is a group of extension methods, built to be used by lombok.
 * For regular methods, see {@link UtilMethods}.
 * <p>
 * Extension methods are methods that are instance methods of a class
 * that are not located within that class.
 *
 * <pre> {@code
 * // required for extension methods to work
 * @ExtensionMethod(UtilExtensionMethods.class)
 * public class Something {
 *     public Something() {
 *         // with lombok
 *         sparkMax.resetAndConfigure(config);
 *         var config = talonFX.getConfigurator().currConfig();
 *         // without lombok
 *         ChargerExtensions.resetAndConfigure(sparkMax, config);
 *         ChargerExtensions.currConfig(talonFX.getConfigurator());
 *     }
 * }
 * } </pre>
 * If you put your own static method here, the very first parameter will act as the
 * extension receiver.
 */
public class UtilExtensionMethods {
    private UtilExtensionMethods() {}
	
	/**
	 * Runs toRun with the receiver, then returns the receiver.
	 * Usage: <code>someValue.also(it -> it.instanceMethod());</code>
	 */
	public static <T> T also(T receiver, Consumer<T> toRun) {
		toRun.accept(receiver);
		return receiver;
	}
	
	/** Configures a spark motor with default ResetMode and PersistMode options. */
	public static REVLibError resetAndConfigure(SparkBase receiver, SparkBaseConfig config) {
		return receiver.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
	}
	
	public static double distanceTo(Pose2d start, Pose2d end) {
		return start.getTranslation().getDistance(end.getTranslation());
	}
	
	public static Pose2d flip(Pose2d receiver, SymmetryStrategy strategy) {
		return new Pose2d(
			flip(receiver.getTranslation(), strategy),
			flip(receiver.getRotation(), strategy)
		);
	}
	
	public static Rotation2d flip(Rotation2d receiver, SymmetryStrategy strategy) {
		return switch (strategy) {
			case VERTICAL -> new Rotation2d(-receiver.getCos(), receiver.getSin());
			case ROTATIONAL -> new Rotation2d(-receiver.getCos(), -receiver.getSin());
			case HORIZONTAL -> new Rotation2d(receiver.getCos(), -receiver.getSin());
		};
	}
	public static Translation2d flip(Translation2d receiver, SymmetryStrategy strategy) {
		return new Translation2d(
			strategy.flipX(receiver.getX()),
			strategy.flipY(receiver.getY())
		);
	}
}
