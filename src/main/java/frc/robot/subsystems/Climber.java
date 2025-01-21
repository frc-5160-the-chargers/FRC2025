package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Climber extends StandardSubsystem {
	@Override
	public Command stopCmd() {
		// TODO
		return Commands.none();
	}
	
	/*
	Todo:
	1. Add a climbUp command that uses PID
	2. PID climbing tuning with TunableDouble
	 */
}
