package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.chargers.hardware.motorcontrol.ChargerTalonFX;
import frc.chargers.hardware.motorcontrol.Motor;
import frc.chargers.hardware.motorcontrol.SimDynamics;
import frc.chargers.utils.data.InputStream;
import frc.chargers.utils.data.TunableValues.TunableNum;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.units.Units.*;
import static frc.chargers.utils.UtilMethods.waitThenRun;

@Logged
public class Climber extends StandardSubsystem {
	private static final TunableNum KG_VOLTS = new TunableNum("climber/kG", -0.4);
	private static final TunableNum KG_START_LIMIT_DEG = new TunableNum("climber/kGStartLimitDeg", 90);
	
	private static final double GEAR_RATIO = 75.0;
	private static final MomentOfInertia MOI = KilogramSquareMeters.of(.003);
	private static final int ID = 7;
	private static final DCMotor MOTOR_KIND = DCMotor.getFalcon500(1);
	private static final TalonFXConfiguration CONFIG = new TalonFXConfiguration();
	
	private final Motor motor = new ChargerTalonFX(ID, true, CONFIG)
		                            .withSim(SimDynamics.of(MOTOR_KIND, GEAR_RATIO, MOI));
	private final DigitalInput limitSwitch = new DigitalInput(2);
	private double voltageReq = 0;
	
	public Climber() {
		waitThenRun(2, () -> motor.encoder().setPositionReading(Degrees.zero()));
		motor.setControlsConfig(Motor.ControlsConfig.EMPTY.withGearRatio(GEAR_RATIO));
	}
	
	public Command setPowerCmd(InputStream output) {
		return this.run(() -> {
			voltageReq = output.get() * 12;
			double positionRad = motor.encoder().positionRad();
			if (positionRad > degreesToRadians(KG_START_LIMIT_DEG.get())) {
				voltageReq += KG_VOLTS.get(); // don't do a cosine gravity term because of the lack of absolute encoder
			}
		    if (voltageReq < 0 && limitSwitch.get()) voltageReq = 0;
			motor.setVoltage(voltageReq);
		})
	       .withName("set voltage(climber)");
	}
	
	@Override
	protected void requestStop() {
		motor.setVoltage(0);
	}
}
