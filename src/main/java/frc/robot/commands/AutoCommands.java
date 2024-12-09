package frc.robot.commands;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.SwerveDrive;
import lombok.RequiredArgsConstructor;

import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.autonomous;

@RequiredArgsConstructor
public class AutoCommands {
	private final AutoFactory autoFactory;
	private final SwerveDrive drivetrain;
	
	static {
		autonomous().and(RobotBase::isSimulation).onTrue(
			Commands.waitSeconds(15.3)
				.finallyDo(() -> DriverStationSim.setEnabled(false))
				.withName("Simulated Auto Ender")
		);
	}
	
	public Command fivePiece() {
		return autoFactory.voidLoop().cmd();
	}
}
