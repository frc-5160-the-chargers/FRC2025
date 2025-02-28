package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.chargers.hardware.motorcontrol.ChargerSpark;
import frc.chargers.hardware.motorcontrol.ChargerSpark.SparkModel;
import frc.chargers.hardware.motorcontrol.Motor;
import frc.chargers.hardware.motorcontrol.SimDynamics;
import frc.chargers.hardware.motorcontrol.SimMotor;

import static edu.wpi.first.units.Units.*;

@Logged
public class Climber extends StandardSubsystem {
	private static final double GEAR_RATIO = 5.0;
	private static final MomentOfInertia MOI = KilogramSquareMeters.of(.003);
	private static final Angle TOLERANCE = Degrees.of(2);
	
	private final TrapezoidProfile motionProfile = new TrapezoidProfile(new Constraints(2.0, 3.0));
	private TrapezoidProfile.State currentState = new TrapezoidProfile.State();
	private final Motor motor;

	public Climber() {
		if (RobotBase.isSimulation()) {
			motor = new SimMotor(SimDynamics.of(DCMotor.getNEO(1), GEAR_RATIO, MOI), null);
		} else {
			motor = new ChargerSpark(5, SparkModel.SPARK_MAX, null);
		}
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
			Commands.runOnce(() -> log("targetAngle", Double.NaN))
		)
	       .withName("set angle(climber)");
	}
	
	@Override
	protected void requestStop() {
		motor.setVoltage(0);
	}
}
