package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.chargers.hardware.motorcontrol.ChargerSpark;
import frc.chargers.hardware.motorcontrol.ChargerSpark.Model;
import frc.chargers.hardware.motorcontrol.Motor;
import frc.chargers.hardware.motorcontrol.SimDynamics;
import frc.chargers.utils.data.InputStream;
import frc.chargers.utils.data.PIDConstants;
import frc.chargers.utils.data.TunableValues.TunableNum;

import java.util.Set;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;

// Currently, a positive angle means pointing down, and a negative one is pointing up
public class CoralIntakePivot extends StandardSubsystem {
	private static final double ELEVATOR_SPEED_LIMIT = 0.4;
	private static final DCMotor MOTOR_KIND = DCMotor.getNeo550(1);
	private static final int MOTOR_ID = 13;
	private static final Angle TOLERANCE = Degrees.of(1.2);
	private static final double GEAR_RATIO = 256 / 3.0;
	private static final MomentOfInertia MOI = KilogramSquareMeters.of(0.012);
	private static final Angle ZERO_OFFSET = Radians.of(-0.951);
	
	private static final double KV = 1 / (MOTOR_KIND.KvRadPerSecPerVolt / GEAR_RATIO);
	private static final ArmFeedforward FEEDFORWARD = RobotBase.isSimulation()
		? new ArmFeedforward(0, 0, KV)
	    : new ArmFeedforward(0, -0.31, KV);
	
	// In rad/sec and rad/sec^2
	private static final double MAX_VEL = (12 - FEEDFORWARD.getKs()) / KV;
	private static final double MAX_ACCEL = 20;
	
	private static final TunableNum KP = new TunableNum("coralIntakePivot/kP", 0.8);
	private static final TunableNum KD = new TunableNum("coralIntakePivot/kD", 0.01);
	private static final TunableNum DEMO_TARGET_DEG = new TunableNum("coralIntakePivot/demoTarget(deg)", 0);
	private static final TunableNum DEMO_VOLTAGE = new TunableNum("coralIntakePivot/demoVoltage", 0);
	
	private static final SparkBaseConfig MOTOR_CONFIG =
		new SparkMaxConfig()
			.smartCurrentLimit(60)
			.idleMode(IdleMode.kBrake)
			.inverted(true)
			.voltageCompensation(12);
	
	static {
		MOTOR_CONFIG.absoluteEncoder.zeroCentered(true);
	}
	
	private final DoubleSupplier elevatorSpeed;
	private final TrapezoidProfile motionProfile = new TrapezoidProfile(new Constraints(MAX_VEL, MAX_ACCEL));
	private TrapezoidProfile.State profileState = new TrapezoidProfile.State();
	@Logged private final Motor motor = new ChargerSpark(MOTOR_ID, Model.SPARK_MAX, MOTOR_CONFIG)
		                                    .withSim(SimDynamics.of(MOTOR_KIND, GEAR_RATIO, MOI), MOTOR_KIND);
	@Logged private Angle target = Degrees.of(Double.NaN);
	@Logged public final Trigger atTarget =
		new Trigger(() -> Math.abs(angleRads() - target.in(Radians)) < TOLERANCE.in(Radians));
	private boolean elevatorWasFast = false;

	public CoralIntakePivot(DoubleSupplier elevatorSpeed) {
		this.elevatorSpeed = elevatorSpeed;
//		waitThenRun(2, () -> motor.encoder().setPositionReading(STARTING_ANGLE));
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
	
	@Logged
	public double gravityCompensationV() {
		return FEEDFORWARD.getKg() * Math.cos(angleRads());
	}
	
	public Command setDemoAngleCmd() {
		return Commands.defer(() -> setAngleCmd(Degrees.of(DEMO_TARGET_DEG.get())), Set.of(this));
	}

	public Command setAngleCmd(Angle target) {
		var goalState = new TrapezoidProfile.State(target.in(Radians), 0);
		return Commands.runOnce(() -> {
			profileState = new TrapezoidProfile.State(angleRads(), motor.encoder().velocityRadPerSec());
			this.elevatorWasFast = false;
			this.target = target;
		})
	       .andThen(
			   this.run(() -> {
				   if (Math.abs(elevatorSpeed.getAsDouble()) > ELEVATOR_SPEED_LIMIT) {
					   requestStop();
					   elevatorWasFast = true;
					   return;
				   } else if (elevatorWasFast) {
					   elevatorWasFast = false;
					   profileState = new TrapezoidProfile.State(angleRads(), 0);
				   }
				   double previousVel = profileState.velocity;
				   profileState = motionProfile.calculate(0.02, profileState, goalState);
				   log("velIsNegative", previousVel < 0);
				   double feedforward = FEEDFORWARD.calculateWithVelocities(
					   angleRads(), previousVel, profileState.velocity
				   );
				   log("feedforward", feedforward);
				   motor.moveToPosition(applyOffset(profileState.position), feedforward);
			   })
	       )
	       .until(atTarget)
	       .finallyDo(this::requestStop)
	       .withName("set angle (pivot)");
	}
	
	public Command setPowerCmd(InputStream controllerInput) {
		return this.run(() -> motor.setVoltage(controllerInput.get() * 12 + gravityCompensationV()))
			       .withName("set power(pivot)");
	}
	
	public Command setDemoVoltageCmd() {
		return this.run(() -> motor.setVoltage(DEMO_VOLTAGE.get()))
			       .withName("set demo volts(pivot)");
	}
	
	@Logged
	public double angleRads() {
		return applyOffset(motor.encoder().positionRad());
	}
	
	private double applyOffset(double radians) {
		return RobotBase.isSimulation() ? radians : radians - ZERO_OFFSET.in(Radians);
	}
	
	@Override
	public void requestStop() {
		motor.setVoltage(gravityCompensationV());
	}
}
