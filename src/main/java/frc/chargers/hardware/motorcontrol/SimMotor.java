package frc.chargers.hardware.motorcontrol;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import org.jetbrains.annotations.Nullable;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

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
	private static int dummyId = 0;
	/** Fetches a dummy ID for simulation-only motors. */
	public static int getDummyId() {
		return dummyId++;
	}
	
	protected final SimDynamics dynamics;
	protected final TalonFXSimState dataInjector;
	protected double currentGearRatio = 1.0;
	
	public SimMotor(SimDynamics dynamics, @Nullable TalonFXConfiguration simConfig) {
		super(SimMotor.getDummyId(), false, simConfig);
		this.dataInjector = super.baseApi.getSimState();
		this.dynamics = dynamics;
		HAL.registerSimPeriodicAfterCallback(this::periodicCallback);
	}
	
	@Override
	public void setControlsConfig(ControlsConfig newConfig) {
		super.setControlsConfig(newConfig);
		currentGearRatio = newConfig.gearRatio();
	}
	
	public SimMotor setOrientation(ChassisReference orientation) {
		dataInjector.Orientation = orientation;
		return this;
	}
	
	@Override
	public SimMotor enablePhoenixPro(boolean useTorqueControl) {
		super.enablePhoenixPro(useTorqueControl);
		return this;
	}
	
	private void periodicCallback() {
		if (RobotBase.isReal()) return;
		dynamics.acceptVolts().accept(MathUtil.clamp(dataInjector.getMotorVoltage(), -12, 12));
		dataInjector.setSupplyVoltage(RobotController.getBatteryVoltage());
		dataInjector.setRawRotorPosition(Radians.of(dynamics.position().getAsDouble() * currentGearRatio));
		dataInjector.setRotorVelocity(RadiansPerSecond.of(dynamics.velocity().getAsDouble() * currentGearRatio));
	}
}
