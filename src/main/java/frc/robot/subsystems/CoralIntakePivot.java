package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.chargers.hardware.motorcontrol.ChargerSpark;
import frc.chargers.hardware.motorcontrol.ChargerSpark.Model;
import frc.chargers.hardware.motorcontrol.Motor;
import frc.chargers.hardware.motorcontrol.SimDynamics;
import frc.chargers.utils.InputStream;
import frc.chargers.utils.PIDConstants;
import frc.chargers.utils.TunableValues.TunableNum;

import java.util.Set;

import static edu.wpi.first.units.Units.*;
import static frc.chargers.utils.UtilMethods.waitThenRun;

// Currently, a positive angle means pointing down, and a negative one is pointing up
public class CoralIntakePivot extends StandardSubsystem {
	private static final Angle STARTING_ANGLE = Degrees.of(-80);
	private static final Angle NAN_ANGLE = Degrees.of(Double.NaN);
	private static final DCMotor MOTOR_KIND = DCMotor.getNeo550(1);
	private static final int MOTOR_ID = 13;
	private static final Angle TOLERANCE = Degrees.of(2.0);
	private static final double GEAR_RATIO = 256.0 / 3.0;
	private static final MomentOfInertia MOI = KilogramSquareMeters.of(0.012);
	
	// In rad/sec and rad/sec^2
	private static final double MAX_VEL = MOTOR_KIND.freeSpeedRadPerSec / GEAR_RATIO;
	private static final double MAX_ACCEL = 20;
	
	private static final TunableNum KP = new TunableNum("coralIntakePivot/kP", 1.2);
	private static final TunableNum KD = new TunableNum("coralIntakePivot/kD", 0.02);
	private static final TunableNum DEMO_ANGLE_DEG = new TunableNum("coralIntakePivot/demoAngle(deg)", 0);
	
	private static final SparkBaseConfig MOTOR_CONFIG =
		new SparkMaxConfig()
			.smartCurrentLimit(60)
			.idleMode(IdleMode.kBrake)
			.inverted(true)
			.voltageCompensation(12);
	
	private final TrapezoidProfile motionProfile = new TrapezoidProfile(new Constraints(MAX_VEL, MAX_ACCEL));
	private TrapezoidProfile.State profileState = new TrapezoidProfile.State();
	@Logged private final Motor motor = new ChargerSpark(MOTOR_ID, Model.SPARK_MAX, MOTOR_CONFIG)
		                                    .withSim(SimDynamics.of(MOTOR_KIND, GEAR_RATIO, MOI), MOTOR_KIND);
	@Logged private Angle target = NAN_ANGLE;
	@Logged public final Trigger atTarget = new Trigger(() -> Math.abs(angleRads() - target.in(Radians)) < TOLERANCE.in(Radians));

	public CoralIntakePivot() {
		waitThenRun(2, () -> motor.encoder().setPositionReading(STARTING_ANGLE));
		setGearRatioAndPID();
		KP.onChange(this::setGearRatioAndPID);
		KD.onChange(this::setGearRatioAndPID);
	}
	
	private void setGearRatioAndPID() {
		motor.setControlsConfig(
			Motor.ControlsConfig.EMPTY
				.withGearRatio(GEAR_RATIO)
				.withPositionPID(new PIDConstants(KP.get(), 0.0, KD.get()))
				.withContinuousInput(true)
		);
	}
	
	public Command setDemoAngleCmd() {
		return Commands.defer(() -> setAngleCmd(Degrees.of(DEMO_ANGLE_DEG.get())), Set.of(this));
	}

	public Command setAngleCmd(Angle target) {
		var goalState = new TrapezoidProfile.State(target.in(Radians), 0);
		return Commands.runOnce(() -> {
			profileState = new TrapezoidProfile.State(motor.encoder().positionRad(), 0);
			this.target = target;
		})
	       .andThen(
			   this.run(() -> {
				   profileState = motionProfile.calculate(0.02, profileState, goalState);
				   motor.moveToPosition(profileState.position, 0);
			   })
	       )
	       .until(atTarget)
	       .finallyDo(this::requestStop)
	       .withName("set angle (pivot)");
	}
	
	public Command setPowerCmd(InputStream controllerInput) {
		return this.run(() -> motor.setVoltage(controllerInput.get() * 12))
			       .withName("set power (pivot)");
	}
	
	public Command resetAngleToStowCmd() {
		return Commands.runOnce(() -> motor.encoder().setPositionReading(STARTING_ANGLE))
			       .withName("reset angle to stow");
	}
	
	public Command resetAngleToZeroCmd() {
		return Commands.runOnce(() -> motor.encoder().setPositionReading(Degrees.zero()))
			       .withName("reset angle to 0");
	}
	
	public double angleRads() {
		return motor.encoder().positionRad();
	}
	
	@Override
	public void requestStop() {
		motor.setVoltage(0);
		this.target = NAN_ANGLE;
	}
}
