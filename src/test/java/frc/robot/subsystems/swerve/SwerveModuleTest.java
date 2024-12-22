package frc.robot.subsystems.swerve;


import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Commands;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.crescendo2024.Arena2024Crescendo;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj2.command.CommandTestingUtil.*;
import static org.junit.jupiter.api.Assertions.assertEquals;

class SwerveModuleTest {
	@BeforeEach
	void setup() {
		assert HAL.initialize(500, 0);
	}

	@Test
	void setAngleTest() {
		var drivetrain = new SwerveDrive("Test drivetrain", SwerveConfigurator.defaultConfig());
		var module = drivetrain.getSwerveModules()[0];
		var scheduler = newCommandScheduler();
		runUntilComplete(
			scheduler,
			Commands.run(() -> {
				module.setAngle(2.0);
			}, drivetrain),
			Seconds.of(3)
		);
		assertEquals(2.0, module.currentState().angle.getRadians(), 0.05);
	}

	@Test
	void setVelocityTest() {
		var drivetrain = new SwerveDrive("Test drivetrain", SwerveConfigurator.defaultConfig());
		SimulatedArena.overrideInstance(new Arena2024Crescendo());
		int ticks = timeToTicks(Seconds.of(3));
		for (int i = 0; i < ticks; i++) {
			SimulatedArena.getInstance().simulationPeriodic();
			for (var module: drivetrain.getSwerveModules()) {
				module.setDesiredState(
					new SwerveModuleState(2.0, Rotation2d.kZero),
					true
				);
			}
		}
		assertEquals(2.0, drivetrain.getSwerveModules()[0].currentState().speedMetersPerSecond, 0.05);
	}
}