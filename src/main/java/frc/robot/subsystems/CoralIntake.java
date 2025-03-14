package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.chargers.hardware.motorcontrol.ChargerSpark;
import frc.chargers.hardware.motorcontrol.ChargerSpark.Model;
import frc.chargers.hardware.motorcontrol.Motor;
import frc.chargers.hardware.motorcontrol.Motor.ControlsConfig;
import frc.chargers.hardware.motorcontrol.SimDynamics;
import frc.chargers.utils.data.InputStream;
import frc.chargers.utils.LaserCanUtil;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

// Constant power when coral in
@Logged
public class CoralIntake extends StandardSubsystem {
	private static final double GEAR_RATIO = 5;
	private static final MomentOfInertia MOI = KilogramSquareMeters.of(0.001);
	
	private static final int MOTOR_ID = 7;
	private static final int LASER_CAN_ID = 1;
	private static final DCMotor MOTOR_KIND = DCMotor.getNeoVortex(1);
	
	private static final double DISTANCE_TOLERANCE_MM = 50;
	private static final double OUTTAKE_VOLTAGE = 4;
	private static final double INTAKE_VOLTAGE = -6;
	private static final double OUTTAKE_DELAY_SECS = 0.8;
	private static final SparkBaseConfig MOTOR_CONFIG =
		new SparkFlexConfig()
			.smartCurrentLimit(60)
			.idleMode(IdleMode.kBrake)
			.inverted(true)
			.voltageCompensation(12.0);
	
	private final Motor motor = new ChargerSpark(MOTOR_ID, Model.SPARK_FLEX, MOTOR_CONFIG)
		                            .withSim(SimDynamics.of(MOTOR_KIND, GEAR_RATIO, MOI), MOTOR_KIND);
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
	/** A trigger that returns true when the intake is outtaking coral. */
	public final Trigger isOuttaking = new Trigger(() -> motor.outputVoltage() > 1.0);
	
	public CoralIntake() {
		motor.setControlsConfig(ControlsConfig.EMPTY.withGearRatio(GEAR_RATIO));
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
			       .andThen(super.stopCmd())
			       .withName("coral intake");
	}
	
	public Command outtakeCmd() {
		return this.run(() -> motor.setVoltage(OUTTAKE_VOLTAGE))
			       .until(hasCoral.negate().and(() -> !runContinuously))
			       .andThen(
					   this.run(() -> motor.setVoltage(OUTTAKE_VOLTAGE)).withTimeout(OUTTAKE_DELAY_SECS),
				       super.stopCmd()
			       )
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
		// set a constant power so coral stays in
		motor.setVoltage(-0.2);
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
