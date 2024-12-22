package frc.robot.subsystems.intake;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.chargers.hardware.motorcontrol.ChargerTalonFX;
import frc.chargers.hardware.motorcontrol.Motor;
import frc.chargers.hardware.motorcontrol.SimMotor;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.wpilibj.RobotBase.isSimulation;

public class DummyIntakeTest extends SubsystemBase {
	private static final double GEAR_RATIO = 16.0;
	
	private final IntakeSimulation mapleSim;
	@Logged private final Motor intakeMotor = isSimulation()
		? new SimMotor(DCMotor.getKrakenX60(1), GEAR_RATIO, KilogramSquareMeters.of(0.004))
	    : new ChargerTalonFX(6, GEAR_RATIO);
	
	public DummyIntakeTest(SwerveDriveSimulation swerveSim) {
		this.mapleSim = new IntakeSimulation(
			"Note", swerveSim,
			Inches.of(27),
			IntakeSide.BACK,
			1
		);
		setDefaultCommand(idleCmd());
	}
	
	public Command intakeCmd() {
		return this.run(() -> {
			mapleSim.startIntake();
			intakeMotor.setVoltage(11);
		});
	}
	
	public Command idleCmd() {
		return this.run(() -> {
			mapleSim.stopIntake();
			intakeMotor.setVoltage(0);
		});
	}
}
