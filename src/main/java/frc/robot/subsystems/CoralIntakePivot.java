package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class CoralIntakePivot extends StandardSubsystem {
	/*
	Todo:
	1. have command factory for moving the pivot to a certain angle
	2. Check Penn State ri3d for pivot setpoints; make that an enum
	3. Add manual control command
	4. Visualize pivot with log calls
	5. Add tuning with TunableDouble
	 */
	
	public Command setAngleCmd(Angle target) {
		return Commands.print("TODO! - Pivot");
	}
	
	public Command idleCmd() { return this.run(() -> {}); }
}
