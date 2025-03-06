package frc.chargers.hardware.motorcontrol;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;

/** Dynamics for simulated motors. */
public record SimDynamics(DoubleSupplier position, DoubleSupplier velocity, DoubleConsumer acceptVolts) {
	public static SimDynamics of(DCMotor motorType, double gearRatio, MomentOfInertia moi) {
		return of(LinearSystemId.createDCMotorSystem(motorType, moi.in(KilogramSquareMeters), gearRatio));
	}
	
	public static SimDynamics of(LinearSystem<N2, N1, N2> plant) {
		var sim = new DCMotorSim(plant, DCMotor.getNEO(1));
		return new SimDynamics(
			sim::getAngularPositionRad,
			sim::getAngularVelocityRadPerSec,
			volts -> {
				sim.setInputVoltage(volts);
				sim.update(0.02);
			}
		);
	}
	
	public static SimDynamics ofElevator(
		DCMotor motorKind, Mass carriageMass,
		double gearRatio, Distance drumRadius, boolean simulateGravity
	) {
		var plant = LinearSystemId.createElevatorSystem(motorKind, carriageMass.in(Kilograms), drumRadius.in(Meters), gearRatio);
		return ofElevator(plant, drumRadius, simulateGravity);
	}
	
	public static SimDynamics ofElevator(LinearSystem<N2, N1, N2> plant, Distance pulleyRadius, boolean simulateGravity) {
		var sim = new ElevatorSim(plant, DCMotor.getNEO(1), 0, Double.POSITIVE_INFINITY, simulateGravity, 0);
		return new SimDynamics(
			() -> sim.getPositionMeters() / pulleyRadius.in(Meters),
			() -> sim.getVelocityMetersPerSecond() / pulleyRadius.in(Meters),
			volts -> {
				sim.setInputVoltage(volts);
				sim.update(0.02);
			}
		);
	}
}
