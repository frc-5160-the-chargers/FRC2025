package frc.chargers.utils;

import choreo.auto.AutoTrajectory;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;
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
	
	/** Gets the distance between 2 Pose2d's. */
	public static double getDistance(Pose2d start, Pose2d end) {
		return start.getTranslation().getDistance(end.getTranslation());
	}
	
	/** "Binds" an alert to a boolean supplier; pushing it to the dashboard when the condition returns true. */
	public static void bind(Alert alert, BooleanSupplier condition) {
		var trigger = condition instanceof Trigger t ? t : new Trigger(condition);
		trigger
			.onTrue(Commands.runOnce(() -> alert.set(true)))
			.onFalse(Commands.runOnce(() -> alert.set(false)));
	}
}
