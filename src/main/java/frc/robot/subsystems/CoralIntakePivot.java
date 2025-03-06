package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
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
import frc.robot.constants.Setpoint;

import java.util.Set;

import static edu.wpi.first.units.Units.*;
import static frc.chargers.utils.UtilMethods.waitThenRun;

// Currently, a positive angle means pointing down, and a negative one is pointing up
public class CoralIntakePivot extends StandardSubsystem {
	private static final int MOTOR_ID = 0;
	private static final Angle TOLERANCE = Degrees.of(2.0);
	private static final double GEAR_RATIO = 256.0 / 3.0;
	private static final MomentOfInertia MOI = KilogramSquareMeters.of(0.012);
	private static final TunableNum KP = new TunableNum("coralIntakePivot/kP", 20);
	private static final TunableNum KD = new TunableNum("coralIntakePivot/kD", 0.1);
	private static final TunableNum DEMO_ANGLE_DEG = new TunableNum("coralIntakePivot/demoAngle(deg)", 0);
	
	private static final SparkMaxConfig MOTOR_CONFIG = new SparkMaxConfig();
	
	@Logged private final Motor motor = new ChargerSpark(MOTOR_ID, Model.SPARK_MAX, MOTOR_CONFIG)
		                                    .withSim(SimDynamics.of(DCMotor.getNeo550(1), GEAR_RATIO, MOI), DCMotor.getNeo550(1));
	@Logged private Angle target = Setpoint.STOW_LOW.wristTarget();
	@Logged public final Trigger atTarget = new Trigger(() -> Math.abs(angleRads() - target.in(Radians)) < TOLERANCE.in(Radians));

	public CoralIntakePivot() {
		waitThenRun(2, () -> motor.encoder().setPositionReading(Setpoint.STOW_LOW.wristTarget()));
		
		setGearRatioAndPID();
		KP.changed.or(KD.changed)
            .onTrue(Commands.runOnce(this::setGearRatioAndPID).ignoringDisable(true));
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
		return this.run(() -> {
			this.target = target;
			motor.moveToPosition(target.in(Radians), 0);
		})
	       .until(atTarget)
	       .finallyDo(this::requestStop)
	       .withName("set angle (pivot)");
	}
	
	
	public Command setPowerCmd(InputStream controllerInput) {
		return this.run(() -> motor.setVoltage(controllerInput.get() * 12))
			       .withName("set power (pivot)");
	}
	
	public double angleRads() {
		return motor.encoder().positionRad();
	}
	
	@Override
	public void requestStop() {
		motor.setVoltage(0);
	}
}
