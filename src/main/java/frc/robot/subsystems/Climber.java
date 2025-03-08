package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.chargers.hardware.motorcontrol.ChargerTalonFX;
import frc.chargers.hardware.motorcontrol.Motor;
import frc.chargers.hardware.motorcontrol.SimDynamics;
import frc.chargers.utils.InputStream;
import frc.chargers.utils.PIDConstants;
import frc.chargers.utils.TunableValues.TunableNum;

import java.util.Set;

import static edu.wpi.first.units.Units.*;
import static frc.chargers.utils.UtilMethods.waitThenRun;

@Logged
public class Climber extends StandardSubsystem {
	private static final double GEAR_RATIO = 75.0;
	private static final MomentOfInertia MOI = KilogramSquareMeters.of(.003);
	private static final Angle TOLERANCE = Degrees.of(2);
	private static final int ID = 7;
	private static final DCMotor MOTOR_KIND = DCMotor.getFalcon500(1);
	private static final TalonFXConfiguration CONFIG = new TalonFXConfiguration();
	private static final PIDConstants PID_CONSTANTS = new PIDConstants(200, 0, 0.1);
	private static final TunableNum DEMO_ANGLE_DEG = new TunableNum("climber/demoAngleDeg", 0);
	
	private final TrapezoidProfile motionProfile = new TrapezoidProfile(new Constraints(2.0, 3.0));
	private TrapezoidProfile.State currentState = new TrapezoidProfile.State();
	private final Motor motor = new ChargerTalonFX(ID, true, CONFIG)
		                            .withSim(SimDynamics.of(MOTOR_KIND, GEAR_RATIO, MOI));
	
	public Climber() {
		waitThenRun(2, () -> motor.encoder().setPositionReading(Degrees.zero()));
		motor.setControlsConfig(
			Motor.ControlsConfig.EMPTY
				.withGearRatio(GEAR_RATIO)
				.withPositionPID(PID_CONSTANTS)
		);
	}
	
	public Command setAngleCmd(Angle target) {
		var targetState = new TrapezoidProfile.State(target.in(Radians), 0);
		return Commands.runOnce(() -> {
			currentState = new TrapezoidProfile.State(motor.encoder().positionRad() , 0);
			log("targetAngle", target);
		}).andThen(
			this.run(() -> {
				currentState = motionProfile.calculate(0.02, currentState, targetState);
				motor.moveToPosition(target.in(Radians));
			}).until(() -> Math.abs(motor.encoder().positionRad() - target.in(Radians)) < TOLERANCE.in(Radians)),
			Commands.runOnce(() -> {
				motor.setVoltage(0);
				log("targetAngle", Double.NaN);
			})
		);
	}
	
	public Command setPowerCmd(InputStream output) {
		return this.run(() -> motor.setVoltage(output.get() * 12))
			       .withName("set voltage(climber)");
	}
	
	public Command climbUp() {
		return setAngleCmd(Degrees.of(90)).withName("Climb up");
	}
	
	public Command climbDown() {
		return setAngleCmd(Degrees.zero()).withName("Climb down");
	}
	
	public Command setDemoAngleCmd() {
		return Commands.defer(() -> setAngleCmd(Degrees.of(DEMO_ANGLE_DEG.get())), Set.of(this))
			       .withName("Set demo angle(climber)");
	}
	
	@Override
	protected void requestStop() {
		motor.setVoltage(0);
	}
}
