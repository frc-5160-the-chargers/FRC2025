package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import frc.chargers.hardware.motorcontrol.Motor;
import frc.chargers.hardware.motorcontrol.SimMotor;


@Logged
public class Climber extends StandardSubsystem {
	private final Motor motor2;
	private final Motor motor1;

	public Climber() {
		motor2 = new SimMotor(
				SimMotor.SimMotorType.DC(DCMotor.getNEO(1), 0.004),
				null);
		motor1 = new SimMotor(
				SimMotor.SimMotorType.DC(DCMotor.getNEO(1), 0.004),
				null);
	}

	public Command setVoltage(double voltage) {
		return this.run(() -> {
			motor2.setVoltage(voltage);
			motor1.setVoltage(voltage);
		});
	}


	@Override
	protected void requestStop() {
		motor2.setVoltage(0);
		motor1.setVoltage(0);
	}

	public Command climbUp() {
		return setVoltage(12);
	}
}
