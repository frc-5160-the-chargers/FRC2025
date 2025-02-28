package frc.chargers.hardware.motorcontrol;

import edu.wpi.first.math.system.plant.DCMotor;
import org.junit.jupiter.api.Test;
import testingutil.StandardUnitTest;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertTrue;

class MotorTest extends StandardUnitTest {
	@Test
	void setVoltageWorks() {
		var simMotor = new SimMotor(SimDynamics.of(DCMotor.getNEO(1), 1.0, KilogramSquareMeters.of(0.004)), null);
		simMotor.setVoltage(5);
		fastForward(null, Seconds.of(2));
		assertTrue(simMotor.encoder().velocityRadPerSec() > 0);
		simMotor.close();
	}
}