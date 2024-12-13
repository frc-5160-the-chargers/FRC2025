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
		if (RobotBase.isSimulation()) {
			autonomous().onTrue(
				Commands.waitSeconds(15.3)
					.finallyDo(() -> DriverStationSim.setEnabled(false))
					.withName("Simulated Auto Ender")
			);
		}
	}
	
	public Command fivePiece() {
		var routine = autoFactory.newRoutine("FivePiece");
		var ampToC1 = routine.trajectory("ampToC1");
		var c1ToScore = routine.trajectory("c1ToScore");
		var c2ToScore = routine.trajectory("c2ToScore");
		var c3ToScore = routine.trajectory("c3ToScore");
		routine.running().onTrue(
			ampToC1.cmd()
		);
		return routine.cmd();
	}
}
