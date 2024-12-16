package frc.chargers.hardware.motorcontrol;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;

/**
 * A Simulated motor that uses TalonFX sim as a backend,
 * while implementing MapleSim's SimulatedMotorController api.
 */
public class SimMotor extends ChargerTalonFX implements SimulatedMotorController {
	private static int currId = 0;
	private final DCMotorSim sim;
	private final TalonFXSimState talonSimApi;
	private final double gearRatio;
	private boolean isMapleSim = false;
	
	public static SimMotor elevator(
		DCMotor motorType,
		double gearRatio,
		MomentOfInertia moi,
		Distance drumRadius
	) {
		return new SimMotor(
			LinearSystemId.createElevatorSystem(motorType, gearRatio, moi.in(KilogramSquareMeters), drumRadius.in(Meters)),
			gearRatio,
			motorType
		);
	}
	
	public SimMotor(DCMotor motorType, double gearRatio, MomentOfInertia moi) {
		this(
			LinearSystemId.createDCMotorSystem(motorType, gearRatio, moi.in(KilogramSquareMeters)),
			gearRatio,
			motorType
		);
	}
	
	public SimMotor(
		LinearSystem<N2, N1, N2> linearSystem,
		double gearRatio,
		DCMotor motorType,
		double... measurementStdDevs
	) {
		super(currId++, gearRatio);
		this.talonSimApi = super.baseMotor.getSimState();
		this.gearRatio = gearRatio;
		this.sim = new DCMotorSim(linearSystem, motorType, measurementStdDevs);
		
		new SubsystemBase("Dummy sim motorType updater") {
			@Override public void simulationPeriodic() { periodicCallback(); }
		};
	}
	
	public SimMotor setOrientation(ChassisReference orientation) {
		this.talonSimApi.Orientation = orientation;
		return this;
	}
	
	@Override
	public Voltage updateControlSignal(
		Angle mechanismAngle,
		AngularVelocity mechanismVelocity,
		Angle encoderAngle,
		AngularVelocity encoderVelocity
	) {
		isMapleSim = true;
		talonSimApi.setSupplyVoltage(12.0);
		talonSimApi.setRawRotorPosition(encoderAngle);
		talonSimApi.setRotorVelocity(encoderVelocity);
		return talonSimApi.getMotorVoltageMeasure();
	}
	
	private void periodicCallback() {
		if (isMapleSim) return;
		var currentBatteryV = RobotController.getBatteryVoltage();
		talonSimApi.setSupplyVoltage(Double.isNaN(currentBatteryV) ? 12.0 : currentBatteryV);
		sim.setInputVoltage(talonSimApi.getMotorVoltage());
		sim.update(0.02);
		talonSimApi.setRawRotorPosition(sim.getAngularPosition().times(gearRatio));
		talonSimApi.setRotorVelocity(sim.getAngularVelocity().times(gearRatio));
	}
}
