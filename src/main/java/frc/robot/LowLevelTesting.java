package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.EpilogueConfiguration;
import edu.wpi.first.wpilibj.TimedRobot;
import monologue.LogLocal;
import monologue.Monologue;

public class LowLevelTesting extends TimedRobot implements LogLocal {
	private final TalonFX drive = new TalonFX(4);
	private final TalonFX steer = new TalonFX(5);
	
	public LowLevelTesting() {
		Monologue.setup(this, new EpilogueConfiguration());
		drive.setNeutralMode(NeutralModeValue.Brake);
		steer.setNeutralMode(NeutralModeValue.Brake);
	}
	
	@Override
	public void autonomousPeriodic() {
		steer.setVoltage(5);
	}
	
	@Override
	public void testPeriodic() {
		drive.setVoltage(5);
	}
	
	@Override
	public void teleopInit() {
		drive.setVoltage(0);
		steer.setVoltage(0);
	}
	
	@Override
	public void robotPeriodic() {
		log("drivePosRot", drive.getPosition().getValueAsDouble());
		log("steerPosRot", steer.getPosition().getValueAsDouble());
	}
}
