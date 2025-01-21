package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.chargers.hardware.motorcontrol.ChargerSpark;
import frc.chargers.hardware.motorcontrol.ChargerSpark.SparkModel;
import frc.chargers.hardware.motorcontrol.Motor;
import frc.chargers.hardware.motorcontrol.Motor.ControlsConfig;
import frc.chargers.hardware.motorcontrol.SimMotor;
import frc.chargers.hardware.motorcontrol.SimMotor.SimMotorType;
import frc.chargers.utils.InputStream;

//@Logged
public class CoralIntake extends StandardSubsystem {
	private static final double GEAR_RATIO = 1.0;
	private static final int ID = -1000;
	
	private final LaserCan laserCan = new LaserCan(0);
	private LaserCan.Measurement laserCanMeasurement = laserCan.getMeasurement();
	private final Motor motor;
	
	public final Trigger hasCoral = new Trigger(
		() -> laserCanMeasurement.status == 0 && laserCanMeasurement.distance_mm < 20
	);
	public final Trigger hasNoCoral = hasCoral.negate();
	
	public CoralIntake() {
		if (RobotBase.isSimulation()) {
			motor = new SimMotor(
				SimMotorType.DC(DCMotor.getNEO(1), 0.004),
				null
			);
		} else {
			var config = new SparkMaxConfig();
			ChargerSpark.optimizeBusUtilizationOn(config);
			config
				.smartCurrentLimit(60)
				.idleMode(IdleMode.kBrake)
				.voltageCompensation(12.0);
			motor = new ChargerSpark(ID, SparkModel.SPARK_MAX, config);
		}
		motor.setControlsConfig(
			ControlsConfig.EMPTY.withGearRatio(GEAR_RATIO)
		);
	}
	
	/**
	 * Returns a Command that sets the elevator motors at a specific percent out.
	 * A value of 1.0 is equivalent to 12 volts(and vise-versa for -1.0).
	 */
	public Command setPowerCmd(InputStream controllerInput) {
		return this.run(() -> motor.setVoltage(controllerInput.get() * 12));
	}
	
	public Command intakeCmd() {
		return this.run(() -> motor.setVoltage(-12)).until(hasCoral);
	}
	
	public Command outtakeCmd() {
		return this.run(() -> motor.setVoltage(8)).until(hasCoral.negate());
	}
	
	@Override
	public Command stopCmd() {
		return this.run(() -> motor.setVoltage(0));
	}
	
	@Override
	public void periodic() {
		var newestMeasurement = laserCan.getMeasurement();
		if (newestMeasurement != null) laserCanMeasurement = newestMeasurement;
	}
	
	@Override
	public void close() throws Exception {
		motor.close();
		laserCan.close();
	}
}
