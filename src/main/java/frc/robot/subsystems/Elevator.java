package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.chargers.hardware.motorcontrol.ChargerTalonFX;
import frc.chargers.hardware.motorcontrol.Motor;
import frc.chargers.hardware.motorcontrol.SimMotor;
import frc.chargers.hardware.motorcontrol.SimMotor.SimMotorType;
import frc.chargers.utils.InputStream;
import frc.chargers.utils.LiveData.NTDouble;
import frc.chargers.utils.PIDConstants;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.disabled;
import static frc.chargers.utils.UtilMethods.tryUntilOk;

public class Elevator extends StandardSubsystem {
	private static final double GEAR_RATIO = 54.0 / 8.0;
	private static final Distance DRUM_RADIUS = Inches.of(2.0);
	private static final Mass ELEVATOR_MASS = Kilograms.of(0.05);
	private static final LinearVelocity MAX_LINEAR_VEL = MetersPerSecond.of(6.0);
	private static final LinearAcceleration MAX_LINEAR_ACCEL = MetersPerSecondPerSecond.of(6.0);
	private static final Angle TOLERANCE = Degrees.of(2.0);
	private static final double ELEVATOR_KP = 8.0;
	private static final double ELEVATOR_KD = 0.01;
	private static final TalonFXConfiguration ELEVATOR_CONFIG = new TalonFXConfiguration();
	private static final int LEFT_MOTOR_ID = 5;
	private static final int RIGHT_MOTOR_ID = 6;

	static {
		ELEVATOR_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
		ELEVATOR_CONFIG.CurrentLimits.StatorCurrentLimit = 60;
		ELEVATOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
	}

	private static class RealElevatorMotor extends ChargerTalonFX {
		public RealElevatorMotor() {
			super(LEFT_MOTOR_ID, true, ELEVATOR_CONFIG);
			var follower = new TalonFX(RIGHT_MOTOR_ID);
			follower.setControl(new Follower(LEFT_MOTOR_ID, false));
			tryUntilOk(follower, () -> follower.getConfigurator().apply(ELEVATOR_CONFIG, 0.01));

			disabled()
					.onTrue(Commands.runOnce(() -> {
						var newConfig = ELEVATOR_CONFIG.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
						super.baseApi.getConfigurator().apply(newConfig);
						follower.getConfigurator().apply(newConfig);
					}))
					.onFalse(Commands.runOnce(() -> {
						var newConfig = ELEVATOR_CONFIG.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
						super.baseApi.getConfigurator().apply(newConfig);
						follower.getConfigurator().apply(newConfig);
					}));
		}
	}

	@Logged
	private final Motor leaderMotor;
	// convert to angular constraints
	private final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(
		new Constraints(
			MAX_LINEAR_VEL.in(MetersPerSecond) / DRUM_RADIUS.in(Meters),
			MAX_LINEAR_ACCEL.in(MetersPerSecondPerSecond) / DRUM_RADIUS.in(Meters)
		)
	);
	private TrapezoidProfile.State profileState = new TrapezoidProfile.State();
	private final NTDouble kPTunable = NTDouble.asTunable("elevator/kP", ELEVATOR_KP);
	private final NTDouble kDTunable = NTDouble.asTunable("elevator/kD", ELEVATOR_KD);
	private final SysIdRoutine sysIdRoutine;
	
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
			leaderMotor = new ChargerTalonFX(LEFT_MOTOR_ID, true, null);
		}

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
		leaderMotor.encoder().setPositionReading(Radians.zero());
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
		return this.run(() -> leaderMotor.setVoltage(controllerInput.get() * 12)).withName("SetPowerCmd(Elevator)");
	}

	public Command moveToHeightCmd(Distance targetHeight) {
		log("targetHeight", targetHeight);
		var radiansTarget = targetHeight.in(Meters) / DRUM_RADIUS.in(Meters);
		var goalState = new TrapezoidProfile.State(radiansTarget, 0.0);
		return this.run(() -> {
			profileState = trapezoidProfile.calculate(0.02, profileState, goalState);
			log("motionProfileState/positionRad", profileState.position);
			leaderMotor.moveToPosition(profileState.position);
		}).until(
				() -> Math.abs(radiansTarget - leaderMotor.encoder().positionRad()) < TOLERANCE.in(Radians)
		);
	}

	public Command passiveLiftCmd(Distance maxHeight) {
		return this.run(() -> leaderMotor.setVoltage(1))
				.until(() -> Math.abs(extensionHeight() - maxHeight.in(Meters)) < 0.1)
				.andThen(stopCmd())
				.withName("PassiveLiftCmd");
	}

	@Override
	public Command stopCmd() {
		return this.runOnce(() -> leaderMotor.setVoltage(0));
	}

	public Command sysIdCmd() {
		return sysIdRoutine.quasistatic(Direction.kForward).andThen(
				sysIdRoutine.quasistatic(Direction.kReverse),
				sysIdRoutine.dynamic(Direction.kForward),
				sysIdRoutine.dynamic(Direction.kReverse)
		);
	}

	@Override
	public void close() {
		leaderMotor.close();
	}
}
