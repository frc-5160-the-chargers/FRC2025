package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.chargers.hardware.motorcontrol.ChargerTalonFX;
import frc.chargers.hardware.motorcontrol.Motor;
import frc.chargers.hardware.motorcontrol.SimMotor;
import frc.chargers.utils.PIDConstants;
import frc.chargers.utils.TunableValues.TunableNum;

import java.util.Set;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

// Currently, a positive angle means pointing up, and a negative one is pointing down
public class CoralIntakePivot extends StandardSubsystem {
	@Logged
	private final Motor leaderMotor;
	private static final int LEFT_MOTOR_ID = 5;
	private static final Angle TOLERANCE = Degrees.of(2.0);
	private static final double GEAR_RATIO = 12.0;
	private static final TunableNum KP = new TunableNum("CoralIntake/kP", 2.0);
	private static final TunableNum KD = new TunableNum("CoralIntake/kD", 0.02);
	private static final TunableNum DEMO_ANGLE_DEG = new TunableNum("CoralIntake/demoAngle(deg)", 0);

	public CoralIntakePivot() {
		if (RobotBase.isSimulation()) {
			leaderMotor = new SimMotor(
				SimMotor.SimMotorType.DC(DCMotor.getKrakenX60(1), 0.025),
				null
			);
		} else {
			leaderMotor = new ChargerTalonFX(LEFT_MOTOR_ID, true, null);
		}
		
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
	
	public Command setDemoAngleCmd() {
		return Commands.defer(() -> setAngleCmd(Degrees.of(DEMO_ANGLE_DEG.get())), Set.of(this));
	}

	public Command setAngleCmd(Angle target) {
		return this.run(() -> {
			leaderMotor.moveToPosition(target.in(Radians));
			log("targetAngle", target);
		}).until(atAngle(target));
	}
	
	public double angleRads() {
		return leaderMotor.encoder().positionRad();
	}
	
	public Trigger atAngle(Angle target) {
		return new Trigger(() -> Math.abs(angleRads() - target.in(Radians)) < TOLERANCE.in(Radians));
	}
	
	@Override
	public Command stopCmd() { return this.runOnce(() -> leaderMotor.setVoltage(0)); }
}
