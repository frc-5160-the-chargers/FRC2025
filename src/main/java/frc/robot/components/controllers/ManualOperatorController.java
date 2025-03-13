package frc.robot.components.controllers;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.chargers.utils.data.InputStream;

import static frc.robot.constants.OtherConstants.IS_DANIEL_COMPUTER;

@Logged
public class ManualOperatorController extends CommandXboxController implements Subsystem {
	private static final int PORT = IS_DANIEL_COMPUTER ? 0 : 1;
	
	public ManualOperatorController() {
		super(PORT);
		this.register();
	}
	
	public Command rumbleCmd(RumbleType type, double time) {
		return this.runOnce(() -> setRumble(type, 0.5))
			       .andThen(
				       Commands.waitSeconds(time),
				       this.runOnce(() -> setRumble(type, 0.5))
			       )
			       .withName("operator controller rumble");
	}
	
	public final InputStream manualElevatorInput =
		InputStream.of(this::getLeftY)
			.deadband(0.1, 1)
			.times(-0.5);
	public final InputStream manualPivotInput =
		InputStream.of(this::getRightY)
			.deadband(0.1, 1)
			.times(-0.3);
	public final InputStream climbUpInput =
		InputStream.of(this::getRightTriggerAxis)
			.deadband(0.1, 1)
			.times(-0.5);
	public final InputStream climbDownInput =
		InputStream.of(this::getLeftTriggerAxis)
			.deadband(0.1, 1)
			.times(0.5);
}
