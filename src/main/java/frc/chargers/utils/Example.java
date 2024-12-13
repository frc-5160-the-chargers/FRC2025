package frc.chargers.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Example {
	public Command driveCmd() {
		return Commands.run(() -> System.out.println("Hello there!!!!!!!!"));
	}
	
	public Command someOtherCmd() {
		return Commands.runOnce(() -> System.out.println("Start"))
			       .andThen(driveCmd());
	}
}
