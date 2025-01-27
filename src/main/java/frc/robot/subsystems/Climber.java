package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.chargers.hardware.motorcontrol.Motor;
import frc.chargers.hardware.motorcontrol.SimMotor;

import static edu.wpi.first.units.Units.Volts;

public class Climber extends StandardSubsystem {
	private final Motor leaderMotor1;
	private final Motor leaderMotor2;

	public Climber() {
		log("init", true);
		leaderMotor1 = new SimMotor(
				SimMotor.SimMotorType.DC(DCMotor.getNEO(1), 0.004),
				null);
		leaderMotor2 = new SimMotor(
				SimMotor.SimMotorType.DC(DCMotor.getNEO(1), 0.004),
				null);
	}

	public Command setVoltage(double voltage) {
		return this.run(() -> {
			leaderMotor1.setVoltage(voltage);
			leaderMotor2.setVoltage(voltage);
		});
	}


	@Override
	public Command stopCmd() {
		return this.runOnce(() -> {
			leaderMotor1.setVoltage(0);
			leaderMotor2.setVoltage(0);
		});
	}
	
	/*
	Todo:
	1. Add a climbUp command that uses PID
	2. PID climbing tuning with TunableDouble
	 */

	public Command climbUp() {
		return this.run(() -> {
			leaderMotor1.setVoltage(100);
			leaderMotor1.setVoltage(100);
		});
	}
}
