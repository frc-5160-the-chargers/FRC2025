package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.LogLocal;

public abstract class StandardSubsystem extends SubsystemBase implements LogLocal, AutoCloseable {
	@Override
	public void close() {
		System.out.println("Warning: subsystem " + getName() + " closed without closing resources.");
	}
}
