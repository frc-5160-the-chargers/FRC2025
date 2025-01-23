package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.UnitTestCmdScheduler;
import org.junit.jupiter.api.Test;
import testingutil.StandardUnitTest;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;

class ElevatorTest extends StandardUnitTest {
	@Test
	void moveToHeight() {
		var scheduler = UnitTestCmdScheduler.create();
		var elevator = new Elevator(false);
		scheduler.schedule(
			elevator.moveToHeightCmd(Meters.of(5.0))
		);
		fastForward(scheduler, Seconds.of(4));
		assertEquals(5.0, elevator.extensionHeight(), 0.05);
		elevator.close();
		scheduler.close();
	}
}