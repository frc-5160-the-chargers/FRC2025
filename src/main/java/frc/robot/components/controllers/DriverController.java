package frc.robot.components.controllers;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.chargers.utils.data.InputStream;

import static frc.robot.constants.OtherConstants.IS_DANIEL_COMPUTER;

@Logged
public class DriverController extends CommandPS5Controller { // does not have a rumble command due to problems
	private static final int PORT = IS_DANIEL_COMPUTER ? 1 : 0;
	
	public DriverController() {
		super(PORT);
	}
	
	private final InputStream slowModeOutput =
		InputStream.of(this::getL2Axis)
			.deadband(0.2, 1)
			.map(it -> 1 - it / 2);
	
	public final InputStream forwardOutput =
		InputStream.of(this::getLeftY)
			.deadband(0.2, 1)
			.times(slowModeOutput)
			.negate();
	public final InputStream strafeOutput =
		InputStream.of(this::getLeftX)
			.deadband(0.2, 1)
			.times(slowModeOutput)
			.negate();
	public final InputStream rotationOutput =
		InputStream.of(this::getRightX)
			.deadband(0.2, 1)
			.times(slowModeOutput)
			.negate();
}
