package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import frc.chargers.hardware.motorcontrol.Motor;
import frc.chargers.hardware.motorcontrol.SimMotor;


@Logged
public class Climber extends StandardSubsystem {
	private final Motor leaderMotor1;
	private final Motor leaderMotor2;

	public Climber() {
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
		return setVoltage(0);
	}

	public Command climbUp() {
		return setVoltage(12);
	}
}
