package frc.chargers.hardware.motorcontrol;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import static edu.wpi.first.units.Units.*;

/**
 * A Simulated motor that uses TalonFX sim as a backend,
 * while implementing MapleSim's SimulatedMotorController api.
 */
public class SimMotor extends ChargerTalonFX {
	private static int dummyId = 0;
	private final LinearSystemSim<N2, N1, N2> sim;
	private final TalonFXSimState talonSimApi;
	private boolean isMapleSim = false;
	
	public static SimMotor elevator(DCMotor motorType, double gearRatio, Mass mass) {
		return new SimMotor(
			new ElevatorSim(
				motorType, gearRatio, mass.in(Kilograms), 1.0,
				Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY,
				true, 0.0
			),
			gearRatio
		);
	}
	
	public static SimMotor singleJointedArm(DCMotor motorType, double gearRatio, MomentOfInertia moi, Distance armLength) {
		return new SimMotor(
			new SingleJointedArmSim(
				motorType, gearRatio, moi.in(KilogramSquareMeters), armLength.in(Meters),
				Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY,
				true, 0.0
			),
			gearRatio
		);
	}
	
	public SimMotor(DCMotor motorType, double gearRatio, MomentOfInertia moi) {
		this(
			new DCMotorSim(
				LinearSystemId.createDCMotorSystem(motorType, moi.in(KilogramSquareMeters), gearRatio),
				motorType
			),
			gearRatio
		);
	}
	
	public SimMotor(LinearSystemSim<N2, N1, N2> sim, double gearRatio) {
		super(dummyId++, gearRatio);
		this.talonSimApi = baseMotor.getSimState();
		this.sim = sim;
		HAL.registerSimPeriodicAfterCallback(this::periodicCallback);
	}
	
	public SimMotor setOrientation(ChassisReference orientation) {
		this.talonSimApi.Orientation = orientation;
		return this;
	}
	
	public SimulatedMotorController getMapleSimApi() {
		return (mechAngle, mechVelocity, encoderAngle, encoderVelocity) -> {
			isMapleSim = true;
			talonSimApi.setSupplyVoltage(12.0);
			talonSimApi.setRawRotorPosition(encoderAngle);
			talonSimApi.setRotorVelocity(encoderVelocity);
			return talonSimApi.getMotorVoltageMeasure();
		};
	}
	
	private void periodicCallback() {
		if (isMapleSim || RobotBase.isReal()) return;
		sim.setInput(MathUtil.clamp(talonSimApi.getMotorVoltage(), -12, 12));
		sim.update(0.02);
		
		talonSimApi.setSupplyVoltage(RobotController.getBatteryVoltage());
		talonSimApi.setRawRotorPosition(Radians.of(sim.getOutput(0) * super.gearRatio));
		talonSimApi.setRotorVelocity(RadiansPerSecond.of(sim.getOutput(1) * super.gearRatio));
	}
}
