package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
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

import static com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters;
import static com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters;
import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import static edu.wpi.first.units.Units.*;
import static frc.chargers.utils.UtilMethods.tryUntilOk;
import static frc.chargers.utils.UtilMethods.waitThenRun;

public class Elevator extends StandardSubsystem {
	private static final TunableNum KP = new TunableNum("elevator/kP", 0.2);
	private static final TunableNum KD = new TunableNum("elevator/kD", 0.012);
	private static final TunableNum DEMO_HEIGHT = new TunableNum("elevator/demoHeight", 0);
	private static final TunableNum DEMO_VOLTAGE = new TunableNum("elevator/demoVoltage", 0);
	
	private static final double GEAR_RATIO = 5.0;
	private static final Distance RADIUS = Inches.of(2 * 0.95);
	private static final Mass CARRIAGE_MASS = Pounds.of(7);
	private static final DCMotor MOTOR_KIND = DCMotor.getNEO(2);
	
	private static final Distance TOLERANCE = Inches.of(0.5);
	private static final Distance COG_LOW_BOUNDARY = Meters.of(0.3);
	private static final Distance MAX_HEIGHT = Meters.of(1.27);
	private static final Distance MIN_HEIGHT = Meters.of(-0.01);
	
	// KV is in volts / (meters/sec)
	private static final double KV = 1 / (MOTOR_KIND.KvRadPerSecPerVolt / GEAR_RATIO * RADIUS.in(Meters));
	// calculate kS by placing robot on its side then running the elevator at tiny voltages until it moves
	// calculate kG by setting voltage until it moves, while upright. Subtract from kS
	private static final ElevatorFeedforward FEEDFORWARD =
		RobotBase.isSimulation()
		    ? new ElevatorFeedforward(0, 0, KV)
			: new ElevatorFeedforward(0.15, 0.44, KV); // confirmed data
	
	private static final LinearVelocity MAX_LINEAR_VEL = MetersPerSecond.of((12 - FEEDFORWARD.getKs()) / KV);
	private static final LinearAcceleration MAX_LINEAR_ACCEL = MetersPerSecondPerSecond.of(5);
	
	// Leader is the right motor
	private static final int LEADER_MOTOR_ID = 27;
	private static final int FOLLOWER_MOTOR_ID = 31;
	
	private static final int CURRENT_LIMIT = 80;
	private static final int SECONDARY_CURRENT_LIMIT = 90;
	
	private static final SparkBaseConfig LEADER_CONFIG =
		new SparkMaxConfig()
			.smartCurrentLimit(CURRENT_LIMIT)
			.secondaryCurrentLimit(SECONDARY_CURRENT_LIMIT)
			.inverted(RobotBase.isReal())
			.idleMode(IdleMode.kBrake);
	private static final SparkBaseConfig FOLLOWER_CONFIG =
		new SparkMaxConfig()
			.smartCurrentLimit(CURRENT_LIMIT)
			.secondaryCurrentLimit(SECONDARY_CURRENT_LIMIT)
			.idleMode(IdleMode.kBrake)
			.inverted(false)
			.follow(LEADER_MOTOR_ID, true);
	
	private final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(
		new Constraints(
			MAX_LINEAR_VEL.in(MetersPerSecond) / RADIUS.in(Meters),
			MAX_LINEAR_ACCEL.in(MetersPerSecondPerSecond) / RADIUS.in(Meters)
		)
	);
	private TrapezoidProfile.State currentSetpoint = new TrapezoidProfile.State();
	
	@Logged private final Motor leaderMotor;
	private final SparkMax followerMotor;
	
	@Logged public final Trigger movingUp;
	@Logged public final Trigger atLowPosition;
	@Logged public final Trigger atLowerLimit;
	@Logged public final Trigger atUpperLimit;
	
	public Elevator() {
		this(false);
	}
	
	// package-private; for unit tests
	Elevator(boolean simulateGravity) {
		leaderMotor = new ChargerSpark(LEADER_MOTOR_ID, Model.SPARK_MAX, LEADER_CONFIG)
			              .withSim(SimDynamics.ofElevator(MOTOR_KIND, CARRIAGE_MASS, GEAR_RATIO, RADIUS, simulateGravity), MOTOR_KIND);
		waitThenRun(2, () -> leaderMotor.encoder().setPositionReading(Radians.zero()));
		
		followerMotor = new SparkMax(FOLLOWER_MOTOR_ID, kBrushless);
		tryUntilOk(followerMotor, () -> followerMotor.configure(FOLLOWER_CONFIG, kResetSafeParameters, kPersistParameters));
		
		movingUp = new Trigger(() -> leaderMotor.encoder().velocityRadPerSec() > 0.1);
		atLowPosition = new Trigger(() -> heightMeters() < COG_LOW_BOUNDARY.in(Meters));
		atLowerLimit = new Trigger(() -> heightMeters() < MIN_HEIGHT.in(Meters) && leaderMotor.outputVoltage() < 0);
		atUpperLimit = new Trigger(() -> heightMeters() > MAX_HEIGHT.in(Meters) && leaderMotor.outputVoltage() > 0);
		
		setGearRatioAndPID();
		KP.onChange(this::setGearRatioAndPID);
		KD.onChange(this::setGearRatioAndPID);
		
		log("maxLinearVel", MAX_LINEAR_VEL);
		log("maxLinearAccel", MAX_LINEAR_ACCEL);
	}
	
	private void setGearRatioAndPID() {
		leaderMotor.setControlsConfig(
			Motor.ControlsConfig.EMPTY
				.withGearRatio(GEAR_RATIO)
				.withPositionPID(new PIDConstants(KP.get(), 0.0, KD.get()))
		);
	}
	
	/**
	 * Returns a trigger that is true when the elevator is at the target height,
	 * with the default tolerance.
	 */
	public Trigger atHeight(Distance height) {
		return atHeight(height, TOLERANCE);
	}
	
	/**
	 * Returns a trigger that is true when the elevator is at the target height,
	 * with a given tolerance.
	 */
	public Trigger atHeight(Distance height, Distance tolerance) {
		return new Trigger(() -> Math.abs(heightMeters() - height.in(Meters)) < TOLERANCE.in(Meters));
	}
	
	@Logged
	public double heightMeters() {
		return leaderMotor.encoder().positionRad() * RADIUS.in(Meters);
	}
	
	@Logged
	public double velocityMPS() {
		return leaderMotor.encoder().velocityRadPerSec() * RADIUS.in(Meters);
	}
	
	public Command setPowerCmd(InputStream controllerInput) {
		return this.run(() -> {
			// not technically mathematically accurate - doesn't really matter for manual control though
			double volts = controllerInput.get() * 12 + FEEDFORWARD.getKg() + FEEDFORWARD.getKs();
			leaderMotor.setVoltage(volts);
		}).withName("set power (elevator)");
	}
	
	public Command moveToDemoHeightCmd() {
		// deferred command re-computes the command at runtime; dw abt this if u dont understand
		return Commands.defer(() -> {
			var demoHeight = DEMO_HEIGHT.get();
			if (demoHeight < 0) return Commands.print("Height < 0; demo request ignored.");
			return moveToHeightCmd(Meters.of(demoHeight));
		}, Set.of(this));
	}
	
	public Command moveToHeightCmd(Distance target) {
		var goalState = new TrapezoidProfile.State(target.in(Meters) / RADIUS.in(Meters), 0.0);
		return Commands.runOnce(() -> {
			currentSetpoint = new TrapezoidProfile.State(leaderMotor.encoder().positionRad(), 0);
			log("targetHeight", target.in(Meters));
		}).andThen(
			// this.run() requires the elevator, while Commands.run/Commands.runOnce dont
			this.run(() -> {
				double previousVelTarget = currentSetpoint.velocity * RADIUS.in(Meters);
				currentSetpoint = trapezoidProfile.calculate(0.02, currentSetpoint, goalState);
				double ffOutput = FEEDFORWARD.calculateWithVelocities(
					previousVelTarget, currentSetpoint.velocity * RADIUS.in(Meters)
				);
				log("motionProfileState/positionRad", currentSetpoint.position);
				log("motionProfileState/velRadPerSec", currentSetpoint.velocity);
				log("feedforward", ffOutput);
				leaderMotor.moveToPosition(currentSetpoint.position, ffOutput);
			}).until(atHeight(target))
		).finallyDo(this::requestStop);
	}
	
	@Override
	protected void requestStop() {
		log("targetHeight", Double.NaN);
		leaderMotor.setVoltage(FEEDFORWARD.getKs() + FEEDFORWARD.getKg());
	}
	
	public Command currentZeroCmd() {
		return this.run(() -> leaderMotor.setVoltage(-0.5))
			       .until(() -> leaderMotor.statorCurrent() > 20)
			       .finallyDo((interrupted) -> {
					   if (!interrupted) {
						   leaderMotor.encoder().setPositionReading(Radians.zero());
						   currentSetpoint = new TrapezoidProfile.State();
					   }
			       })
			       .withName("current zero command(Elevator)");
	}
	
	public Command setDemoVoltageCmd() {
		return this.run(() -> leaderMotor.setVoltage(DEMO_VOLTAGE.get()));
	}
	
	@Override
	public void periodic() {
		log("follower/tempCelsius", followerMotor.getMotorTemperature());
		log("follower/statorCurrent", followerMotor.getOutputCurrent());
		if (atUpperLimit.getAsBoolean() || atLowerLimit.getAsBoolean()) requestStop();
	}
	
	@Override
	public void close() {
		leaderMotor.close();
		followerMotor.close();
	}
}
