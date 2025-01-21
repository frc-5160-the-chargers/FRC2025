package frc.chargers.hardware.motorcontrol;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.jetbrains.annotations.Nullable;

import static edu.wpi.first.units.Units.*;

/**
 * A Simulated motor that uses TalonFX sim as a backend,
 * while implementing MapleSim's SimulatedMotorController api.
 * <h4>
 *     Note: due to some issues with the TalonFX api, sim motors
 *     that share an id with a ChargerTalonFX can be buggy.
 *     Just for safety, make sure that real motors are not instantiated in sim.
 * </h4>
 */
public class SimMotor extends ChargerTalonFX {
	@FunctionalInterface
	public interface SimMotorType {
		LinearSystemSim<N2, N1, N2> createSim(double gearRatio);
		
		static SimMotorType DC(DCMotor motorKind, double moiKgMetersSquared) {
			return gearRatio -> new DCMotorSim(
				LinearSystemId.createDCMotorSystem(motorKind, moiKgMetersSquared, gearRatio),
				motorKind
			);
		}
		
		static SimMotorType elevator(DCMotor motorKind, Mass mass) {
			return gearRatio -> new ElevatorSim(
				motorKind, gearRatio, mass.in(Kilograms), 1.0,
				Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY,
				true, 0.0
			);
		}
		
		static SimMotorType singleJointedArm(DCMotor motorKind, double moiKgMetersSquared, Distance armLength) {
			return gearRatio -> new SingleJointedArmSim(
				motorKind, gearRatio, moiKgMetersSquared, armLength.in(Meters),
				Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY,
				true, 0.0
			);
		}
	}
	
	protected final SimMotorType motorType;
	protected final TalonFXSimState talonSimApi;
	
	private static int dummyId = 0;
	protected LinearSystemSim<N2, N1, N2> sim;
	protected double currentGearRatio;
	protected boolean isMapleSim = false;
	
	public SimMotor(SimMotorType motorType, @Nullable TalonFXConfiguration simConfig) {
		super(dummyId++, false, simConfig);
		this.talonSimApi = baseApi.getSimState();
		this.motorType = motorType;
		this.sim = motorType.createSim(1.0); // default to a gear ratio of 1.0
		HAL.registerSimPeriodicAfterCallback(this::periodicCallback);
	}
	
	@Override
	public void setControlsConfig(ControlsConfig newConfig) {
		super.setControlsConfig(newConfig);
		currentGearRatio = newConfig.gearRatio();
		sim = motorType.createSim(newConfig.gearRatio());
	}
	
	public SimMotor withOrientation(ChassisReference orientation) {
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
	
	@Override
	public SimMotor enablePhoenixPro(boolean useTorqueControl) {
		super.enablePhoenixPro(useTorqueControl);
		return this;
	}
	
	private void periodicCallback() {
		if (isMapleSim || RobotBase.isReal()) return;
		sim.setInput(MathUtil.clamp(talonSimApi.getMotorVoltage(), -12, 12));
		sim.update(0.02);
		
		talonSimApi.setSupplyVoltage(RobotController.getBatteryVoltage());
		talonSimApi.setRawRotorPosition(Radians.of(sim.getOutput(0) * currentGearRatio));
		talonSimApi.setRotorVelocity(RadiansPerSecond.of(sim.getOutput(1) * currentGearRatio));
	}
}
