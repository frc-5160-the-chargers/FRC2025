package frc.chargers.hardware.motorcontrol;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.chargers.hardware.encoders.EncoderIO;

import static edu.wpi.first.units.Units.*;

@Logged
public interface MotorIO {
	/** Creates an instance of a MotorIO with a spark max. */
	static <M extends SparkBase> SparkIO<M> of(M baseMotor, double gearRatio) {
		return new SparkIO<>(baseMotor, gearRatio);
	}
	
	/** Creates an instance of a MotorIO with a TalonFX. */
	static TalonFXIO of(TalonFX baseMotor, double gearRatio) {
		return new TalonFXIO(baseMotor, gearRatio);
	}
	
	/** Creates an instance of a simulated MotorIO. */
	static SimMotorIO ofSim(
		DCMotor motor, double gearRatio,
		MomentOfInertia moi, double... measurementStdDevs
	) {
		motor = motor.withReduction(gearRatio);
		return new SimMotorIO(
			LinearSystemId.createDCMotorSystem(motor, moi.in(KilogramSquareMeters), 1.0),
			motor, measurementStdDevs
		);
	}
	
	/** Creates an instance of a simulated MotorIO. */
	static SimMotorIO ofSim(
		LinearSystem<N2, N1, N2> linearSystem,
		DCMotor motor, double... measurementStdDevs
	) {
		return new SimMotorIO(linearSystem, motor, measurementStdDevs);
	}
	
	EncoderIO encoder();
	double outputVoltage();
	double statorCurrent();
	double tempCelsius();
	
	void setVoltage(double volts);
	void setVelocity(double velocityRadPerSec, double ffVolts);
	void moveToPosition(double positionRads, double ffVolts);
	void setTorqueCurrent(double currentAmps);
	void setCoastMode(boolean on);
	
	void setPositionPID(double p, double i, double d);
	void setVelocityPID(double p, double i, double d);
	void enableContinuousInput();
	
	default void setVelocity(AngularVelocity velocity, double ffVolts) {
		setVelocity(velocity.in(RadiansPerSecond), ffVolts);
	}
	
	default void moveToPosition(Angle position, double ffVolts) {
		moveToPosition(position.in(Radians), ffVolts);
	}
	default void moveToPosition(double angleRads) {
		moveToPosition(angleRads, 0);
	}
	default void moveToPosition(Angle angle) {
		moveToPosition(angle.in(Radians), 0);
	}
	
	default void setPositionPID(PIDConstants constants) { setPositionPID(constants.kP, constants.kI, constants.kD); }
	default void setVelocityPID(PIDConstants constants) { setVelocityPID(constants.kP, constants.kI, constants.kD); }
	
	default double supplyCurrent() {
		return statorCurrent() * outputVoltage() / 12.0;
	}
}
