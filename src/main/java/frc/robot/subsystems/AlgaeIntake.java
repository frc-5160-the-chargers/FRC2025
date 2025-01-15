package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import frc.chargers.hardware.motorcontrol.Motor;
import frc.chargers.hardware.motorcontrol.SimMotor;
import frc.chargers.hardware.motorcontrol.SimMotor.SimMotorType;
import frc.chargers.utils.InputStream;

@Logged
public class AlgaeIntake extends StandardSubsystem {
	private final Motor leftMotor = new SimMotor(
		SimMotorType.DC(DCMotor.getNEO(1), 0.025),
		null
	);
	private final Motor rightMotor = new SimMotor(
		SimMotorType.DC(DCMotor.getNEO(1), 0.025),
		null
	);
	
	public Command setPowerCmd(InputStream controllerInput) {
		return this.run(() -> {
			leftMotor.setVoltage(controllerInput.get() * 12);
			rightMotor.setVoltage(controllerInput.get() * -12);
		});
	}
	
	public Command outtakeCmd() {
		return setPowerCmd(() -> 0.7);
	}
	
	public Command intakeCmd() {
		return setPowerCmd(() -> -0.95);
	}
	
	public Command idleCmd() {
		return setPowerCmd(() -> 0);
	}
	
	@Override
	public void close() {
		leftMotor.close();
		rightMotor.close();
	}
}
