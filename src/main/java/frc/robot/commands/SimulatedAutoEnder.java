package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A command that ends auto(in sim) after 15.3 seconds.
 * Uses System.nanoTime() instead, as Timer.getTimestamp() is simulated
 * (which means lagging in sim will cause the auto to be prematurely cancelled).
 */
public class SimulatedAutoEnder extends Command {
	private double startTime = 0.0;
	
	@Override
	public void initialize() {
		startTime = System.nanoTime();
	}
	
	@Override
	public boolean isFinished() {
		return RobotBase.isSimulation() && (System.nanoTime() - startTime) / 1e9 > 15.3;
	}
	
	@Override
	public void end(boolean interrupted) {
		DriverStationSim.setEnabled(false);
	}
}
