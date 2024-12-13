package frc.chargers.utils;

import edu.wpi.first.wpilibj.Alert;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

/**
 * An alert that can have multiple causes. Utilizes a function to generate an error message from a
 * list of causes.
 */
public class MultiAlert extends Alert {
	private final Function<List<String>, String> textGenerator;
	private final List<String> causes = new ArrayList<>();
	
	public MultiAlert(String group, Function<List<String>, String> textGenerator, AlertType type) {
		super(group, textGenerator.apply(List.of()), type);
		this.textGenerator = textGenerator;
	}
	
	public MultiAlert(Function<List<String>, String> textGenerator, AlertType type) {
		super(textGenerator.apply(List.of()), type);
		this.textGenerator = textGenerator;
	}
	
	/**
	 * Adds an error causer to this alert, and pushes the alert to networktables if it is not
	 * already present.
	 *
	 * @param name The name of the error causer
	 */
	public void addCause(String name) {
		causes.add(name);
		setText(textGenerator.apply(causes));
		set(true);
	}
}
