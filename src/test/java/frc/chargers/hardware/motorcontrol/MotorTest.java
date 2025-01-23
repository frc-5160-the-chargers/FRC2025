package frc.chargers.hardware.motorcontrol;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.chargers.hardware.motorcontrol.SimMotor.SimMotorType;
import org.junit.jupiter.api.Test;
import testingutil.StandardUnitTest;

import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertTrue;

class MotorTest extends StandardUnitTest {
	@Test
	void setVoltageWorks() {
		var simMotor = new SimMotor(SimMotorType.DC(DCMotor.getNEO(1), 0.004), null);
		simMotor.setVoltage(5);
		fastForward(null, Seconds.of(2));
		assertTrue(simMotor.encoder().velocityRadPerSec() > 0);
		simMotor.close();
	}
}