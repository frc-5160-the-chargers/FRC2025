package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
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
import frc.robot.CompetitionRobot.SharedState;

import java.util.Set;

import static edu.wpi.first.units.Units.*;

// Currently, a positive angle means pointing down, and a negative one is pointing up
public class CoralIntakePivot extends StandardSubsystem {
	private static final DCMotor MOTOR_KIND = DCMotor.getNeo550(1);
	private static final int MOTOR_ID = 13;
	private static final Angle TOLERANCE = Degrees.of(1.5);
	private static final double GEAR_RATIO = 256 / 3.0;
	private static final MomentOfInertia MOI = KilogramSquareMeters.of(0.012);
	private static final Angle ZERO_OFFSET = RobotBase.isSimulation() ? Radians.zero() : Radians.of(1.622);
	
	private static final double KV = 1 / (MOTOR_KIND.KvRadPerSecPerVolt / GEAR_RATIO);
	private static final SimpleMotorFeedforward FF_EQUATION = new SimpleMotorFeedforward(0.05, KV);
	private static final Voltage NO_CORAL_KG = Volts.of(-0.32);
	private static final Voltage WITH_CORAL_KG = Volts.of(-0.46);
	
	// In rad/sec and rad/sec^2
	private static final double MAX_VEL = (12 - FF_EQUATION.getKs()) / KV;
	private static final double MAX_ACCEL = 25;
	
	private static final TunableNum KP = new TunableNum("coralIntakePivot/kP", 0.64);
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
		MOTOR_CONFIG.absoluteEncoder
			.zeroCentered(true)
			.setSparkMaxDataPortConfig();
	}
	
	private final SharedState sharedState;
	private final TrapezoidProfile motionProfile = new TrapezoidProfile(new Constraints(MAX_VEL, MAX_ACCEL));
	private TrapezoidProfile.State profileState = new TrapezoidProfile.State();
	@Logged private final Motor motor = new ChargerSpark(MOTOR_ID, Model.SPARK_MAX, MOTOR_CONFIG)
		                                    .withAbsoluteEncoder()
		                                    .withSim(SimDynamics.of(MOTOR_KIND, GEAR_RATIO, MOI), MOTOR_KIND);
	@Logged private Angle target = Degrees.of(Double.NaN);
	@Logged private double feedforwardV = 0.0;
	@Logged public final Trigger atTarget =
		new Trigger(() -> Math.abs(angleRads() - target.in(Radians)) < TOLERANCE.in(Radians));

	public CoralIntakePivot(SharedState sharedState) {
		this.sharedState = sharedState;
		setGearRatioAndPID();
		KP.onChange(this::setGearRatioAndPID);
		KD.onChange(this::setGearRatioAndPID);
	}
	
	private void setGearRatioAndPID() {
		motor.setControlsConfig(
			Motor.ControlsConfig.EMPTY.withPositionPID(new PIDConstants(KP.get(), 0.0, KD.get()))
		);
	}
	
	@Logged
	public double gravityCompensationV() {
		if (RobotBase.isSimulation()) return 0.0;
		var kg = sharedState.hasCoralDelayed.getAsBoolean() ? WITH_CORAL_KG : NO_CORAL_KG;
		return kg.in(Volts) * Math.cos(angleRads());
	}
	
	public Command setDemoAngleCmd() {
		return Commands.defer(() -> setAngleCmd(Degrees.of(DEMO_TARGET_DEG.get())), Set.of(this));
	}

	public Command setAngleCmd(Angle target) {
		var goalState = new TrapezoidProfile.State(target.in(Radians), 0);
		return Commands.runOnce(() -> {
			profileState = new TrapezoidProfile.State(angleRads(), motor.encoder().velocityRadPerSec());
			this.target = target;
		})
	       .andThen(
			   this.run(() -> {
				   profileState = motionProfile.calculate(0.02, profileState, goalState);
				   log("profileState/position", profileState.position);
				   log("profileState/velocity", profileState.velocity);
				   feedforwardV = FF_EQUATION.calculate(profileState.velocity);
				   feedforwardV += gravityCompensationV();
				   motor.moveToPosition(profileState.position + ZERO_OFFSET.in(Radians), feedforwardV);
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
		return motor.encoder().positionRad() - ZERO_OFFSET.in(Radians);
	}
	
	@Override
	public void requestStop() {
		motor.setVoltage(gravityCompensationV());
		feedforwardV = gravityCompensationV();
	}
}
