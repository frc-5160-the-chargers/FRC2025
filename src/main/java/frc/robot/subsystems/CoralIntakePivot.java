package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
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
import frc.chargers.hardware.motorcontrol.SimMotor;
import frc.chargers.utils.InputStream;
import frc.chargers.utils.PIDConstants;
import frc.chargers.utils.TunableValues.TunableNum;
import frc.robot.constants.Setpoint;

import java.util.Set;

import static edu.wpi.first.units.Units.*;
import static frc.chargers.utils.UtilMethods.waitThenRun;

// Currently, a positive angle means pointing down, and a negative one is pointing up
public class CoralIntakePivot extends StandardSubsystem {
	private static final int MOTOR_ID = 5;
	private static final Angle TOLERANCE = Degrees.of(2.0);
	private static final double GEAR_RATIO = 16.0;
	private static final MomentOfInertia MOI = KilogramSquareMeters.of(0.03);
	private static final TunableNum KP = new TunableNum("coralIntakePivot/kP", 0.7);
	private static final TunableNum KD = new TunableNum("coralIntakePivot/kD", 0.1);
	private static final TunableNum DEMO_ANGLE_DEG = new TunableNum("coralIntakePivot/demoAngle(deg)", 0);
	
	@Logged private final Motor motor;
	@Logged private Angle target = Setpoint.STOW_LOW.wristTarget();
	@Logged public final Trigger atTarget = new Trigger(() -> Math.abs(angleRads() - target.in(Radians)) < TOLERANCE.in(Radians));

	public CoralIntakePivot() {
		if (RobotBase.isSimulation()) {
			motor = new SimMotor(SimDynamics.of(DCMotor.getNeo550(1), GEAR_RATIO, MOI), null);
		} else {
			var config = new SparkMaxConfig();
			ChargerSpark.optimizeBusUtilizationOn(config);
			config.voltageCompensation(12);
			motor = new ChargerSpark(MOTOR_ID, Model.SPARK_MAX, config);
		}
		waitThenRun(2, () -> motor.encoder().setPositionReading(Setpoint.STOW_LOW.wristTarget()));
		
		setGearRatioAndPID();
		KP.changed.or(KD.changed)
            .onTrue(Commands.runOnce(this::setGearRatioAndPID));
	}
	
	private void setGearRatioAndPID() {
		motor.setControlsConfig(
			Motor.ControlsConfig.EMPTY
				.withGearRatio(GEAR_RATIO)
				.withPositionPID(new PIDConstants(KP.get(), 0.0, KD.get()))
		);
	}
	
	public Command setDemoAngleCmd() {
		return Commands.defer(() -> setAngleCmd(Degrees.of(DEMO_ANGLE_DEG.get())), Set.of(this));
	}

	public Command setAngleCmd(Angle target) {
		return this.run(() -> {
			this.target = target;
			motor.moveToPosition(target.in(Radians));
		}).until(atTarget);
	}
	
	public Command setPowerCmd(InputStream controllerInput) {
		return this.run(() -> motor.setVoltage(controllerInput.get() * 12));
	}
	
	public double angleRads() {
		return motor.encoder().positionRad();
	}
	
	@Override
	public void requestStop() {
		motor.setVoltage(0);
	}
}
