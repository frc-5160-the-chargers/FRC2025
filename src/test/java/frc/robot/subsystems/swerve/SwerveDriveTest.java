package frc.robot.subsystems.swerve;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandTestingUtil;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static edu.wpi.first.units.Units.Seconds;

class SwerveDriveTest {
	@BeforeEach
	void setup() {
		assert HAL.initialize(500, 0);
	}
	
	@Test
	void driveForward() throws Exception {
		SwerveDrive swerveDrive = new SwerveDrive(SwerveConfigurator.defaultConfig());
		var scheduler = CommandTestingUtil.newCommandScheduler();
		
		CommandTestingUtil.runUntilComplete(
			scheduler,
			swerveDrive.driveCmd(() -> 1.0, () -> 0.0, () -> 0.0, true),
			Seconds.of(5)
		);
		
		scheduler.close();
		swerveDrive.close();
	}
	
}