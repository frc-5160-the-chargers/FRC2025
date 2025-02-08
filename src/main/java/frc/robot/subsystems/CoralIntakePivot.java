package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.chargers.hardware.motorcontrol.ChargerTalonFX;
import frc.chargers.hardware.motorcontrol.Motor;
import frc.chargers.hardware.motorcontrol.SimMotor;
import frc.chargers.utils.PIDConstants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

public class CoralIntakePivot extends StandardSubsystem {
	/*
	Todo:
	1. have command factory for moving the pivot to a certain angle
	2. Check Penn State ri3d for pivot setpoints; make that an enum
	3. Add manual control command
	4. Visualize pivot with log calls
	5. Add tuning with TunableDouble
	 */
	@Logged
	private final Motor leaderMotor;
	private static final int LEFT_MOTOR_ID = 5;
	private static final Angle TOLERANCE = Degrees.of(2.0);
	private static final double GEAR_RATIO = 0.0;
	private static final double kP = 0.0;
	private static final double kD = 0.0;

	public CoralIntakePivot() {
		log("init", true);


		if (RobotBase.isSimulation()) {
			leaderMotor = new SimMotor(
					SimMotor.SimMotorType.DC(DCMotor.getKrakenX60(2), 0.004),
					null
			);
		} else {
			leaderMotor = new ChargerTalonFX(LEFT_MOTOR_ID, true, null);
		}

		leaderMotor.setControlsConfig(
			Motor.ControlsConfig.EMPTY
					.withGearRatio(GEAR_RATIO)
					.withPositionPID(new PIDConstants(kP, 0.0, kP))
		);
	}

	public Command setAngleCmd(Angle target) {
		log("targetAngle", target);

		return this.run(() -> {
			leaderMotor.moveToPosition(target.in(Radians));
		}).until(
				() -> Math.abs(target.in(Radians) - leaderMotor.encoder().positionRad()) < TOLERANCE.in(Radians)
		);
	}
	
	@Override
	public Command stopCmd() { return this.runOnce(() -> leaderMotor.setVoltage(0)); }
}
