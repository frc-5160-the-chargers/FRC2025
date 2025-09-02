//package frc.robot.subsystems;
//
//import edu.wpi.first.wpilibj2.command.UnitTestCmdScheduler;
//import frc.robot.CompetitionRobot.SharedState;
//import org.junit.jupiter.api.Test;
//import testingutil.StandardUnitTest;
//
//import static edu.wpi.first.units.Units.Meters;
//import static edu.wpi.first.units.Units.Seconds;
//import static org.junit.jupiter.api.Assertions.assertEquals;
//
//class ElevatorTest extends StandardUnitTest {
//	@Test
//	void moveToHeight() {
//		var scheduler = UnitTestCmdScheduler.create();
//		var elevator = new Elevator(new SharedState());
//		scheduler.schedule(
//			elevator.moveToHeightCmd(Meters.of(5.0))
//		);
//		fastForward(scheduler, Seconds.of(4));
//		assertEquals(5.0, elevator.heightMeters(), 0.05);
//		elevator.close();
//		scheduler.close();
//	}
//}