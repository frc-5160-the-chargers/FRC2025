package frc.robot.subsystems.swerve;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandTestingUtil;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;

class SwerveDriveTest {
	@BeforeEach
	void setup() {
		assert HAL.initialize(500, 0);
	}
	
	@Test
	void ensureRobotDoesntDrift() throws Exception {
		SwerveDrive swerveDrive = new SwerveDrive(SwerveConfigurator.DEFAULT_CONFIG);
		var scheduler = CommandTestingUtil.newCommandScheduler();
		
		// initial angle is 0
		CommandTestingUtil.runUntilComplete(
			scheduler,
			swerveDrive.driveCmd(() -> 1.0, () -> 0.0, () -> 0.0, true),
			Seconds.of(5)
		);
		
		assertEquals(swerveDrive.getPose().getRotation().getRadians(), 0.0, 0.2);
		
		scheduler.close();
		swerveDrive.close();
	}
	
}