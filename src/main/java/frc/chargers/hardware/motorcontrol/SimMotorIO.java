package frc.chargers.hardware.motorcontrol;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.chargers.hardware.encoders.EncoderIO;

public class SimMotorIO implements MotorIO {
	private final DCMotorSim sim;
	private final EncoderIO encoderIO = new EncoderIO() {
		@Override
		public double positionRad() {
			return sim.getAngularPositionRad();
		}
		
		@Override
		public double velocityRadPerSec() {
			return sim.getAngularVelocityRadPerSec();
		}
	};
	private double currVoltage = 0.0;
	private final PIDController positionPID = new PIDController(0, 0, 0);
	private final PIDController velocityPID = new PIDController(0, 0, 0);
	
	public SimMotorIO(LinearSystem<N2, N1, N2> linearSystem, DCMotor motor, double... measurementStdDevs) {
		this.sim = new DCMotorSim(linearSystem, motor, measurementStdDevs);
		if (RobotBase.isReal()) return;
		// allows sim to update implicitly
		new SubsystemBase() {
			@Override public void periodic() { sim.update(0.02); }
		};
	}
	
	@Override
	public EncoderIO encoder() { return encoderIO; }
	
	@Override
	public double outputVoltageVolts() { return currVoltage; }
	
	@Override
	public double currentDrawAmps() { return sim.getCurrentDrawAmps(); }
	
	@Override
	public double tempCelsius() { return 0; }
	
	@Override
	public void setVoltage(double volts) {
		currVoltage = volts;
		sim.setInputVoltage(volts);
	}
	
	@Override
	public void spinAtVelocity(double velocityRadPerSec, double ffVolts) {
		setVoltage(velocityPID.calculate(velocityRadPerSec, encoderIO.velocityRadPerSec()) + ffVolts);
	}
	
	@Override
	public void moveToPosition(double positionRads, double ffVolts) {
		setVoltage(positionPID.calculate(positionRads, encoderIO.positionRad()) + ffVolts);
	}
	
	@Override
	public void setTorqueCurrent(double currentAmps) {
		System.out.println("Setting Torque current is currently not available.");
	}
	
	@Override
	public void setCoastMode(boolean on) {}
	
	@Override
	public void setPositionPID(double p, double i, double d) { positionPID.setPID(p, i, d); }
	
	@Override
	public void setVelocityPID(double p, double i, double d) { velocityPID.setPID(p, i, d); }
}
