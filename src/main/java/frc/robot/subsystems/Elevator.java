package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
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
import frc.chargers.hardware.motorcontrol.ChargerTalonFX;
import frc.chargers.hardware.motorcontrol.Motor;
import frc.chargers.hardware.motorcontrol.SimMotor;
import frc.chargers.hardware.motorcontrol.SimMotor.SimMotorType;
import frc.chargers.utils.InputStream;
import frc.chargers.utils.PIDConstants;
import frc.chargers.utils.TunableValues.TunableNum;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.disabled;
import static frc.chargers.utils.UtilMethods.tryUntilOk;
import static monologue.Monologue.GlobalLog;

public class Elevator extends StandardSubsystem {
	private static final double GEAR_RATIO = 54.0 / 8.0;
	private static final Distance DRUM_RADIUS = Inches.of(2.0);
	private static final Mass ELEVATOR_MASS = Kilograms.of(0.05);
	private static final LinearVelocity MAX_LINEAR_VEL = MetersPerSecond.of(6.0);
	private static final LinearAcceleration MAX_LINEAR_ACCEL = MetersPerSecondPerSecond.of(6.0);
	private static final Angle TOLERANCE = Degrees.of(0.5);
	private static final Distance COG_LOW_BOUNDARY = Meters.of(0.2);
	private static final double ELEVATOR_KP = 7.0;
	private static final double ELEVATOR_KD = 0.1;
	private static final TalonFXConfiguration ELEVATOR_CONFIG = new TalonFXConfiguration();
	private static final int LEFT_MOTOR_ID = 5;
	private static final int RIGHT_MOTOR_ID = 6;

	static {
		ELEVATOR_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
		ELEVATOR_CONFIG.CurrentLimits.StatorCurrentLimit = 60;
		ELEVATOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
	}
	
	/**
	 * A controller for the leader and follower motors of the elevator.
	 * The left motor is the leader, while the right motor is the follower.
	 */
	private static class ElevatorMotorGroup extends ChargerTalonFX {
		private final TalonFX follower = new TalonFX(RIGHT_MOTOR_ID);
		
		public ElevatorMotorGroup(Elevator elevator) {
			super(LEFT_MOTOR_ID, true, ELEVATOR_CONFIG);
			tryUntilOk(follower, () -> follower.getConfigurator().apply(ELEVATOR_CONFIG, 0.01));
			follower.setControl(new Follower(LEFT_MOTOR_ID, false));
			
			disabled()
				.onTrue(Commands.runOnce(() -> super.baseApi.setNeutralMode(NeutralModeValue.Coast)))
				.onFalse(Commands.runOnce(() -> super.baseApi.setNeutralMode(NeutralModeValue.Brake)));
			
			elevator.followerMotorLogger = () -> {
				GlobalLog.log("elevator/follower/tempCelsius", follower.getDeviceTemp().getValueAsDouble());
				GlobalLog.log("elevator/follower/current", follower.getStatorCurrent().getValueAsDouble());
			};
		}
	}
	
	// convert to angular constraints
	private final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(
		new Constraints(
			MAX_LINEAR_VEL.in(MetersPerSecond) / DRUM_RADIUS.in(Meters),
			MAX_LINEAR_ACCEL.in(MetersPerSecondPerSecond) / DRUM_RADIUS.in(Meters)
		)
	);
	private TrapezoidProfile.State profileState = new TrapezoidProfile.State();
	private final TunableNum kPTunable = new TunableNum("elevator/kP", ELEVATOR_KP);
	private final TunableNum kDTunable = new TunableNum("elevator/kD", ELEVATOR_KD);
	private Runnable followerMotorLogger = () -> {};
	private final SysIdRoutine sysIdRoutine;
	
	@Logged private final Motor leaderMotor;
	@Logged public final Trigger movingUp;
	@Logged public final Trigger readyForMovement;
	
	public Elevator() {
		this(true);
	}

	// package-private; for unit tests
	Elevator(boolean simulateGravity) {
		log("init", true);
		if (RobotBase.isSimulation()) {
			leaderMotor = new SimMotor(
				SimMotorType.elevator(DCMotor.getKrakenX60(2), ELEVATOR_MASS, simulateGravity),
				null
			);
		} else {
			leaderMotor = new ElevatorMotorGroup(this);
		}
		leaderMotor.encoder().setPositionReading(Radians.zero());
		movingUp = new Trigger(() -> leaderMotor.encoder().velocityRadPerSec() > 0.1);
		readyForMovement = movingUp.negate().and(() -> extensionHeight() < COG_LOW_BOUNDARY.in(Meters));
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
		setMotorCommonConfig();
		kPTunable.changed().or(kDTunable.changed())
				.onTrue(Commands.runOnce(this::setMotorCommonConfig));
	}

	private void setMotorCommonConfig() {
		leaderMotor.setControlsConfig(
				Motor.ControlsConfig.EMPTY
						.withGearRatio(GEAR_RATIO)
						.withPositionPID(new PIDConstants(kPTunable.get(), 0.0, kDTunable.get()))
		);
	}

	@Logged
	public double extensionHeight() {
		return leaderMotor.encoder().positionRad() * DRUM_RADIUS.in(Meters);
	}

	public Command setPowerCmd(InputStream controllerInput) {
		return this.run(() -> leaderMotor.setVoltage(controllerInput.get() * 12))
			       .withName("SetPowerCmd(Elevator)");
	}
	
	public Command syncStateCmd() {
		return Commands.runOnce(() -> profileState = new TrapezoidProfile.State(extensionHeight(), 0))
			       .withName("SyncStateCmd(Elevator)");
	}

	public Command moveToHeightCmd(Distance targetHeight) {
		log("targetHeight", targetHeight);
		var radiansTarget = targetHeight.in(Meters) / DRUM_RADIUS.in(Meters);
		var goalState = new TrapezoidProfile.State(radiansTarget, 0.0);
		return syncStateCmd().andThen(
			this.run(() -> {
				profileState = trapezoidProfile.calculate(0.02, profileState, goalState);
				log("motionProfileState/positionRad", profileState.position);
				leaderMotor.moveToPosition(profileState.position);
			}).until(() -> Math.abs(radiansTarget - leaderMotor.encoder().positionRad()) < TOLERANCE.in(Radians))
		).withName("MoveToHeightCmd");
	}

	public Command passiveLiftCmd(Distance maxHeight) {
		return this.run(() -> leaderMotor.setVoltage(1))
				.until(() -> Math.abs(extensionHeight() - maxHeight.in(Meters)) < 0.1)
				.andThen(stopCmd())
				.withName("PassiveLiftCmd");
	}

	@Override
	public Command stopCmd() {
		return this.runOnce(() -> leaderMotor.setVoltage(0)).withName("StopCmd(Elevator)");
	}

	public Command sysIdCmd() {
		return sysIdRoutine.quasistatic(Direction.kForward).andThen(
			sysIdRoutine.quasistatic(Direction.kReverse),
			sysIdRoutine.dynamic(Direction.kForward),
			sysIdRoutine.dynamic(Direction.kReverse)
        ).withName("ElevatorSysId");
	}
	
	@Override
	public void periodic() {
		followerMotorLogger.run();
		// logs relative positions for advantagescope visualization
		double currentHeight = extensionHeight();
		log("stage1Position", Pose3d.kZero);
		log("stage2Position", new Pose3d(0, 0, Math.max(currentHeight - 0.4, 0.0), Rotation3d.kZero));
		log("stage3Position", new Pose3d(0, 0, currentHeight, Rotation3d.kZero));
	}

	@Override
	public void close() {
		leaderMotor.close();
	}
}
