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
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;

/** Dynamics for simulated motors. */
public record SimDynamics(DoubleSupplier position, DoubleSupplier velocity, DoubleConsumer acceptVolts) {
	public static SimDynamics of(DCMotor motorType, double gearRatio, MomentOfInertia moi) {
		return of(LinearSystemId.createDCMotorSystem(motorType, moi.in(KilogramSquareMeters), gearRatio));
	}
	
	public static SimDynamics of(LinearSystem<N2, N1, N2> plant) {
		// motor type is only used in current calcs, thus it is replaced with dummy values
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
	
	public static SimDynamics of(
		DCMotor motorType, double gearRatio, MomentOfInertia moi,
		Distance pivotLength, boolean addGravity
	) {
		return of(
			LinearSystemId.createSingleJointedArmSystem(motorType, moi.in(KilogramSquareMeters), gearRatio),
			pivotLength, addGravity
		);
	}
	
	public static SimDynamics of(LinearSystem<N2, N1, N2> plant, Distance pivotLength, boolean addGravity) {
		// gearing and motor type are only used in current calcs, thus they are replaced with dummy values
		var sim = new SingleJointedArmSim(
			plant, DCMotor.getNEO(1), 1.0, pivotLength.in(Meters),
			Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, addGravity, 0
		);
		return new SimDynamics(
			sim::getAngleRads,
			sim::getVelocityRadPerSec,
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
		var sim = new ElevatorSim(
			motorKind, gearRatio, carriageMass.in(Kilograms),
			drumRadius.in(Meters), 0, Double.POSITIVE_INFINITY,
			simulateGravity, 0
		);
		return new SimDynamics(
			() -> sim.getPositionMeters() / drumRadius.in(Meters),
			() -> sim.getVelocityMetersPerSecond() / drumRadius.in(Meters),
			volts -> {
				sim.setInputVoltage(volts);
				sim.update(0.02);
			}
		);
	}
}
