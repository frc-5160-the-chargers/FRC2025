package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.chargers.hardware.motorcontrol.ChargerSpark;
import frc.chargers.hardware.motorcontrol.ChargerSpark.SparkModel;
import frc.chargers.hardware.motorcontrol.Motor;
import frc.chargers.hardware.motorcontrol.Motor.ControlsConfig;
import frc.chargers.hardware.motorcontrol.SimDynamics;
import frc.chargers.hardware.motorcontrol.SimMotor;
import frc.chargers.utils.InputStream;
import frc.chargers.utils.LaserCanUtil;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

@Logged
public class CoralIntake extends StandardSubsystem {
	private static final double GEAR_RATIO = 5;
	private static final MomentOfInertia MOI = KilogramSquareMeters.of(.004);
	
	private static final int MOTOR_ID = -1000;
	private static final int LASER_CAN_ID = 1;
	
	private static final double DISTANCE_TOLERANCE_MM = 50;
	private static final double OUTTAKE_VOLTAGE = 12;
	private static final double INTAKE_VOLTAGE = -12;
	private static final double DELAY_SECS = 0.5;
	
	private final Motor motor;
	private final LaserCan laserCan = new LaserCan(LASER_CAN_ID);
	private LaserCan.Measurement laserCanMeasurement = LaserCanUtil.NULL_OP_MEASUREMENT;
	private boolean hasCoralInSim = false;
	/** Whether outtakeCmd() and intakeCmd() should end when a gamepiece is detected. */
	public boolean runContinuously = false;
	/** A trigger that returns true when the intake detects coral. */
	public final Trigger hasCoral = new Trigger(
		() -> (RobotBase.isSimulation() && hasCoralInSim) ||
			      (laserCanMeasurement.status == 0 && laserCanMeasurement.distance_mm < DISTANCE_TOLERANCE_MM)
	);
	
	public CoralIntake() {
		if (RobotBase.isSimulation()) {
			motor = new SimMotor(SimDynamics.of(DCMotor.getNEO(1), GEAR_RATIO, MOI), null);
		} else {
			var config = new SparkMaxConfig();
			ChargerSpark.optimizeBusUtilizationOn(config);
			config
				.smartCurrentLimit(60)
				.idleMode(IdleMode.kBrake)
				.voltageCompensation(12.0);
			motor = new ChargerSpark(MOTOR_ID, SparkModel.SPARK_MAX, config);
		}
		motor.setControlsConfig(
			ControlsConfig.EMPTY.withGearRatio(GEAR_RATIO)
		);
	}
	
	public double velocityRadPerSec() {
		return motor.encoder().velocityRadPerSec();
	}
	
	public Command setHasCoralInSimCmd(boolean hasCoral) {
		return Commands.runOnce(() -> hasCoralInSim = hasCoral);
	}
	
	/**
	 * Returns a Command that sets the elevator motors at a specific percent out.
	 * A value of 1.0 is equivalent to 12 volts(and vise-versa for -1.0).
	 */
	public Command setPowerCmd(InputStream controllerInput) {
		return this.run(() -> motor.setVoltage(controllerInput.get() * 12))
			       .withName("coral set power");
	}
	
	public Command intakeCmd() {
		return this.run(() -> motor.setVoltage(INTAKE_VOLTAGE))
			       .until(hasCoral)
			       .andThen(this.run(() -> motor.setVoltage(INTAKE_VOLTAGE)).withTimeout(DELAY_SECS))
			       .withName("coral intake");
	}
	
	public Command outtakeCmd() {
		return this.run(() -> motor.setVoltage(OUTTAKE_VOLTAGE))
			       .until(hasCoral.negate().and(() -> !runContinuously))
			       .andThen(this.run(() -> motor.setVoltage(OUTTAKE_VOLTAGE)).withTimeout(DELAY_SECS))
			       .withName("coral outtake");
	}
	
	public Command intakeForeverCmd() {
		return this.run(() -> motor.setVoltage(INTAKE_VOLTAGE)).withName("coral intake(Forever)");
	}
	
	public Command outtakeForeverCmd() {
		return this.run(() -> motor.setVoltage(OUTTAKE_VOLTAGE)).withName("coral outtake(Forever)");
	}
	
	@Override
	protected void requestStop() {
		motor.setVoltage(0);
	}
	
	@Override
	public void periodic() {
		laserCanMeasurement = laserCan.getMeasurement();
		if (laserCanMeasurement == null) {
			laserCanMeasurement = LaserCanUtil.NULL_OP_MEASUREMENT;
		}
	}
	
	@Override
	public void close() throws Exception {
		motor.close();
		laserCan.close();
	}
}
