package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.chargers.hardware.motorcontrol.ChargerSpark;
import frc.chargers.hardware.motorcontrol.ChargerSpark.SparkModel;
import frc.chargers.hardware.motorcontrol.Motor;
import frc.chargers.hardware.motorcontrol.SimMotor;
import frc.chargers.hardware.motorcontrol.SimMotor.SimMotorType;
import frc.chargers.utils.InputStream;
import frc.chargers.utils.PIDConstants;
import frc.chargers.utils.TunableValues.TunableNum;

import java.util.Set;

import static com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters;
import static com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters;
import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.disabled;
import static frc.chargers.utils.UtilMethods.tryUntilOk;

public class Elevator extends StandardSubsystem {
	private static final TunableNum KP = new TunableNum("elevator/kP", RobotBase.isSimulation() ? 2000 : 800);
	private static final TunableNum KD = new TunableNum("elevator/kD", RobotBase.isSimulation() ? 500 : 30);
	private static final TunableNum DEMO_HEIGHT = new TunableNum("elevator/testHeight", 0);
	
	private static final double GEAR_RATIO = 54.0 / 8.0;
	private static final Distance DRUM_RADIUS = Inches.of(1.0);
	private static final Mass ELEVATOR_MASS = Kilograms.of(0.05);
	
	private static final LinearVelocity MAX_LINEAR_VEL = MetersPerSecond.of(2);
	private static final LinearAcceleration MAX_LINEAR_ACCEL = MetersPerSecondPerSecond.of(12.0);
	
	private static final Distance TOLERANCE = Inches.of(0.5);
	private static final Distance COG_LOW_BOUNDARY = Meters.of(0.7);
	
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
			MAX_LINEAR_VEL.in(MetersPerSecond) / DRUM_RADIUS.in(Meters),
			MAX_LINEAR_ACCEL.in(MetersPerSecondPerSecond) / DRUM_RADIUS.in(Meters)
		)
	);
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
			leaderMotor = new SimMotor(SimMotorType.elevator(DCMotor.getNEO(2), ELEVATOR_MASS, simulateGravity), simConfig);
		} else {
			leaderMotor = new ChargerSpark(LEADER_MOTOR_ID, SparkModel.SPARK_MAX, LEADER_CONFIG);
		}
		leaderMotor.encoder().setPositionReading(Radians.zero());
		
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
	
	/** Returns a trigger that returns true when the elevator is at a target height. */
	public Trigger atHeight(Distance height) {
		return new Trigger(() -> Math.abs(extensionHeight() - height.in(Meters)) < TOLERANCE.in(Meters));
	}
	
	@Logged
	public double extensionHeight() {
		return leaderMotor.encoder().positionRad() * DRUM_RADIUS.in(Meters);
	}
	
	public Command setPowerCmd(InputStream controllerInput) {
		return this.run(() -> leaderMotor.setVoltage(controllerInput.get() * 12))
			       .withName("SetPowerCmd(Elevator)");
	}
	
	public Command moveToDemoHeightCmd() {
		// deferred command re-computes the command at runtime; dw abt this if u dont understand
		return Commands.defer(() -> moveToHeightCmd(Meters.of(DEMO_HEIGHT.get())), Set.of(this));
	}
	
	public Command moveToHeightCmd(Distance targetHeight) {
		var radiansTarget = targetHeight.in(Meters) / DRUM_RADIUS.in(Meters);
		var goalState = new TrapezoidProfile.State(radiansTarget, 0.0);
		return Commands.runOnce(() -> {
			profileState = new TrapezoidProfile.State(leaderMotor.encoder().positionRad() , 0);
			log("targetHeight", targetHeight);
		}).andThen(
			// this.run() requires the elevator, while Commands.run/Commands.runOnce dont
			this.run(() -> {
				profileState = trapezoidProfile.calculate(0.02, profileState, goalState);
				log("motionProfileState/positionRad", profileState.position);
				leaderMotor.moveToPosition(profileState.position);
			}).until(atHeight(targetHeight)),
			Commands.runOnce(() -> {
				log("targetHeight", Double.NaN);
				requestStop();
			})
		).withName("MoveToHeightCmd");
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
        ).withName("ElevatorSysId");
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
			       .withName("CurrentZeroCmd(Elevator)");
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
