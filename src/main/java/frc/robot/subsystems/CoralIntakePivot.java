package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.chargers.hardware.motorcontrol.ChargerTalonFX;
import frc.chargers.hardware.motorcontrol.Motor;
import frc.chargers.hardware.motorcontrol.SimDynamics;
import frc.chargers.hardware.motorcontrol.SimMotor;
import frc.chargers.utils.InputStream;
import frc.chargers.utils.PIDConstants;
import frc.chargers.utils.TunableValues.TunableNum;

import java.util.Set;

import static edu.wpi.first.units.Units.*;

// Currently, a positive angle means pointing up, and a negative one is pointing down
public class CoralIntakePivot extends StandardSubsystem {
	private static final int MOTOR_ID = 5;
	private static final Angle TOLERANCE = Degrees.of(2.0);
	private static final double GEAR_RATIO = 12.0;
	private static final MomentOfInertia MOI = KilogramSquareMeters.of(.025);
	private static final TunableNum KP = new TunableNum("coralIntakePivot/kP", 2.0);
	private static final TunableNum KD = new TunableNum("coralIntakePivot/kD", 0.02);
	private static final TunableNum DEMO_ANGLE_DEG = new TunableNum("coralIntakePivot/demoAngle(deg)", 0);
	
	@Logged private final Motor motor;

	public CoralIntakePivot() {
		if (RobotBase.isSimulation()) {
			motor = new SimMotor(SimDynamics.of(DCMotor.getKrakenX60(1), GEAR_RATIO, MOI), null);
		} else {
			motor = new ChargerTalonFX(MOTOR_ID, true, null);
		}
		
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
			motor.moveToPosition(target.in(Radians));
			log("targetAngle", target);
		}).until(atAngle(target));
	}
	
	public Command setPowerCmd(InputStream controllerInput) {
		return this.run(() -> motor.setVoltage(controllerInput.get() * 12));
	}
	
	public double angleRads() {
		return motor.encoder().positionRad();
	}
	
	public Trigger atAngle(Angle target) {
		return new Trigger(() -> Math.abs(angleRads() - target.in(Radians)) < TOLERANCE.in(Radians));
	}
	
	@Override
	public void requestStop() {
		motor.setVoltage(0);
	}
}
