package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.chargers.hardware.motorcontrol.Motor;
import frc.chargers.hardware.motorcontrol.SimMotor;
import frc.chargers.utils.PIDConstants;
import monologue.LogLocal;
import monologue.Monologue;

import static edu.wpi.first.units.Units.Radians;

@Logged
public class Tests extends TimedRobot implements LogLocal {
	private Motor motor = new SimMotor(
		SimMotor.SimMotorType.DC(DCMotor.getNEO(1), 0.004),
		null
	);
	
	
	
	public Tests() {
		Epilogue.bind(this);
		Monologue.setup(this, Epilogue.getConfig());
		motor.setCommonConfig(
			Motor.CommonConfig.EMPTY
				.withPositionPID(new PIDConstants(7, 0, 0))
				.withGearRatio(150.0 / 7.0)
				.withContinuousInput(true)
		);
	}
	
	@Override
	public void autonomousPeriodic() {
		motor.moveToPosition(Radians.of(5.0));
	}
}
