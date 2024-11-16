package frc.chargers.utils;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoCommand {
	/** Use AutoCommand.of() instead. */
	private AutoCommand() {}
	
	public static Command of(Command defaultAuto, Command... otherAutos) {
		var chooser = new SendableChooser<Command>();
		var requirements = defaultAuto.getRequirements();
		chooser.setDefaultOption("Default: " + defaultAuto.getName(), defaultAuto);
		for (Command cmd : otherAutos) {
			chooser.addOption(cmd.getName(), cmd);
			requirements.retainAll(cmd.getRequirements());
		}
		SmartDashboard.putData("Autonomous Command", chooser);
		return Commands.defer(chooser::getSelected, requirements).withName("Autonomous Command");
	}
}
