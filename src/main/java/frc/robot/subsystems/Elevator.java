package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.chargers.hardware.motorcontrol.ChargerSpark;
import frc.chargers.hardware.motorcontrol.ChargerSpark.Model;
import frc.chargers.hardware.motorcontrol.Motor;
import frc.chargers.hardware.motorcontrol.SimDynamics;
import frc.chargers.hardware.motorcontrol.SimMotor;
import frc.chargers.utils.InputStream;
import frc.chargers.utils.PIDConstants;
import frc.chargers.utils.TunableValues.TunableNum;

import java.util.Set;
import java.util.function.BiFunction;

import static com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters;
import static com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters;
import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.disabled;
import static frc.chargers.utils.UtilMethods.tryUntilOk;
import static frc.chargers.utils.UtilMethods.waitThenRun;

public class Elevator extends StandardSubsystem {
	private static final TunableNum KP = new TunableNum("elevator/kP", 3000);
	private static final TunableNum KD = new TunableNum("elevator/kD", 350);
	private static final TunableNum DEMO_HEIGHT = new TunableNum("elevator/testHeight", 0);
	
	// verified values
	private static final double GEAR_RATIO = 5.0;
	private static final Distance RADIUS = Inches.of(0.95);
	private static final Mass CARRIAGE_MASS = Pounds.of(7);
	private static final DCMotor MOTOR_KIND = DCMotor.getNEO(2);
	
	private static final LinearVelocity MAX_LINEAR_VEL = MetersPerSecond.of(2);
	private static final LinearAcceleration MAX_LINEAR_ACCEL = MetersPerSecondPerSecond.of(18.0);
	
	private static final Distance TOLERANCE = Inches.of(0.5);
	private static final Distance COG_LOW_BOUNDARY = Meters.of(0.5);
	
	private static final int LEADER_MOTOR_ID = 5;
	private static final int FOLLOWER_MOTOR_ID = 6;
	
	private static final int CURRENT_LIMIT = 80;
	private static final int SECONDARY_CURRENT_LIMIT = 90;
	
	private static final SparkBaseConfig LEADER_CONFIG =
		new SparkMaxConfig()
			.smartCurrentLimit(CURRENT_LIMIT)
			.secondaryCurrentLimit(SECONDARY_CURRENT_LIMIT)
			.idleMode(IdleMode.kBrake);
	private static final SparkBaseConfig FOLLOWER_CONFIG =
		new SparkMaxConfig()
			.smartCurrentLimit(CURRENT_LIMIT)
			.secondaryCurrentLimit(SECONDARY_CURRENT_LIMIT)
			.idleMode(IdleMode.kBrake)
			.follow(LEADER_MOTOR_ID);
	
	static {
		ChargerSpark.optimizeBusUtilizationOn(LEADER_CONFIG, FOLLOWER_CONFIG);
	}
	
	// convert to angular constraints
	private final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(
		new Constraints(
			MAX_LINEAR_VEL.in(MetersPerSecond) / RADIUS.in(Meters),
			MAX_LINEAR_ACCEL.in(MetersPerSecondPerSecond) / RADIUS.in(Meters)
		)
	);
	private final BiFunction<Distance, LinearVelocity, Voltage> feedforwardFn;
	private TrapezoidProfile.State profileState = new TrapezoidProfile.State();
	private final SysIdRoutine sysIdRoutine;
	
	@Logged private final Motor leaderMotor;
	private final SparkMax followerMotor;
	
	@Logged public final Trigger movingUp;
	@Logged public final Trigger atLowPosition;
	
	public Elevator() {
		this(false);
	}
	
	// package-private; for unit tests
	Elevator(boolean simulateGravity) {
		if (RobotBase.isSimulation()) {
			var simConfig = new TalonFXConfiguration();
			simConfig.CurrentLimits.StatorCurrentLimitEnable = true;
			simConfig.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT;
			leaderMotor = new SimMotor(
				SimDynamics.ofElevator(MOTOR_KIND, CARRIAGE_MASS, GEAR_RATIO, RADIUS, simulateGravity),
				simConfig
			);
		} else {
			leaderMotor = new ChargerSpark(LEADER_MOTOR_ID, Model.SPARK_MAX, LEADER_CONFIG);
		}
		waitThenRun(2, () -> leaderMotor.encoder().setPositionReading(Radians.zero()));
		
		var plantInversionFF = new LinearPlantInversionFeedforward<>(
			LinearSystemId.createElevatorSystem(
				MOTOR_KIND, CARRIAGE_MASS.in(Kilograms), RADIUS.in(Meters), GEAR_RATIO
			),
			0.02
		);
		feedforwardFn = (targetPosition, targetVelocity) -> Volts.of(
			plantInversionFF.calculate(
				VecBuilder.fill(targetPosition.in(Meters), targetVelocity.in(MetersPerSecond))
			).get(0, 0)
		);
		
		followerMotor = new SparkMax(RobotBase.isSimulation() ? SimMotor.getDummyId() : FOLLOWER_MOTOR_ID, kBrushless);
		tryUntilOk(followerMotor, () -> followerMotor.configure(FOLLOWER_CONFIG.follow(LEADER_MOTOR_ID), kResetSafeParameters, kPersistParameters));
		disabled()
			.onTrue(Commands.runOnce(() -> {
				leaderMotor.setCoastMode(true);
				followerMotor.configure(FOLLOWER_CONFIG.idleMode(IdleMode.kCoast), kResetSafeParameters, kPersistParameters);
			}))
			.onFalse(Commands.runOnce(() -> {
				leaderMotor.setCoastMode(false);
				followerMotor.configure(FOLLOWER_CONFIG.idleMode(IdleMode.kBrake), kResetSafeParameters, kPersistParameters);
			}));
		
		movingUp = new Trigger(() -> leaderMotor.encoder().velocityRadPerSec() > 0.1);
		atLowPosition = new Trigger(() -> extensionHeight() < COG_LOW_BOUNDARY.in(Meters));
		
		sysIdRoutine = new SysIdRoutine(
			new SysIdRoutine.Config(
				null, null, null,
				state -> log("sysIdRoutineState", state.toString())
			),
			new SysIdRoutine.Mechanism(
				voltage -> leaderMotor.setVoltage(voltage.in(Volts)),
				null,
				this,
				"Elevator routine"
			)
		);
		
		setGearRatioAndPID();
		KP.changed.or(KD.changed)
			.onTrue(Commands.runOnce(this::setGearRatioAndPID));
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
		return new Trigger(() -> Math.abs(extensionHeight() - height.in(Meters)) < TOLERANCE.in(Meters));
	}
	
	@Logged
	public double extensionHeight() {
		return leaderMotor.encoder().positionRad() * RADIUS.in(Meters);
	}
	
	public Command setPowerCmd(InputStream controllerInput) {
		return this.run(() -> leaderMotor.setVoltage(controllerInput.get() * 12))
			       .withName("set power command(Elevator)");
	}
	
	public Command moveToDemoHeightCmd() {
		// deferred command re-computes the command at runtime; dw abt this if u dont understand
		return Commands.defer(() -> {
			var demoHeight = DEMO_HEIGHT.get();
			if (demoHeight < 0) return Commands.print("Height < 0; demo request ignored.");
			return moveToHeightCmd(Meters.of(demoHeight));
		}, Set.of(this));
	}
	
	public Command moveToHeightCmd(Distance targetHeight) {
		var radiansTarget = targetHeight.in(Meters) / RADIUS.in(Meters);
		var goalState = new TrapezoidProfile.State(radiansTarget, 0.0);
		return Commands.runOnce(() -> {
			profileState = new TrapezoidProfile.State(leaderMotor.encoder().positionRad() , 0);
			log("targetHeight", targetHeight);
		}).andThen(
			// this.run() requires the elevator, while Commands.run/Commands.runOnce dont
			this.run(() -> {
				profileState = trapezoidProfile.calculate(0.02, profileState, goalState);
				var feedforward = feedforwardFn.apply(
					RADIUS.times(profileState.position),
					RADIUS.times(profileState.velocity).per(Second)
				);
				log("motionProfileState/positionRad", profileState.position);
				log("feedforward", feedforward);
				leaderMotor.moveToPosition(profileState.position, feedforward.in(Volts));
			}).until(atHeight(targetHeight)),
			Commands.runOnce(() -> {
				log("targetHeight", Double.NaN);
				requestStop();
			})
		);
	}
	
	@Override
	protected void requestStop() {
		leaderMotor.setVoltage(0);
	}
	
	public Command sysIdCmd() {
		return sysIdRoutine.quasistatic(Direction.kForward).andThen(
			sysIdRoutine.quasistatic(Direction.kReverse),
			sysIdRoutine.dynamic(Direction.kForward),
			sysIdRoutine.dynamic(Direction.kReverse)
        ).withName("elevator sysid");
	}
	
	public Command currentZeroCmd() {
		return this.run(() -> leaderMotor.setVoltage(-0.5))
			       .until(() -> leaderMotor.statorCurrent() > 20)
			       .finallyDo((interrupted) -> {
					   if (!interrupted) {
						   leaderMotor.encoder().setPositionReading(Radians.zero());
						   profileState = new TrapezoidProfile.State();
					   }
			       })
			       .withName("current zero command(Elevator)");
	}
	
	@Override
	public void periodic() {
		log("follower/tempCelsius", followerMotor.getMotorTemperature());
		log("follower/statorCurrent", followerMotor.getOutputCurrent());
	}
	
	@Override
	public void close() {
		leaderMotor.close();
		followerMotor.close();
	}
}
