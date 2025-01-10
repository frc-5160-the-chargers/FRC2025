package frc.robot.subsystems.elevator;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.chargers.hardware.motorcontrol.ChargerSpark;
import frc.chargers.hardware.motorcontrol.Motor;
import monologue.LogLocal;


public class Elevator extends SubsystemBase implements LogLocal {
	// ChargerSpark.max(), ChargerSpark.flex(), new ChargerTalonFX(), new SimMotor
	// Motor interface
	//
	
	@Logged
	Motor stuff = ChargerSpark.max(5, null);
	
}
