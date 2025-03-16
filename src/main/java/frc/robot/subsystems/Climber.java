package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.chargers.hardware.motorcontrol.ChargerSpark;
import frc.chargers.hardware.motorcontrol.ChargerSpark.Model;
import frc.chargers.hardware.motorcontrol.Motor;
import frc.chargers.hardware.motorcontrol.SimDynamics;
import frc.chargers.utils.data.InputStream;
import frc.chargers.utils.data.TunableValues.TunableNum;

import static com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters;
import static com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters;
import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static frc.chargers.utils.UtilMethods.tryUntilOk;

// Convention: + is forward, - is backward
public class Climber extends StandardSubsystem {
	private static final TunableNum KG_VOLTS = new TunableNum("climber/kGVolts", 0);
	private static final TunableNum KG_START_LIMIT_DEG = new TunableNum("climber/kGStartLimitDeg", 90);
	private static final TunableNum STARTING_POS_DEG = new TunableNum("climber/startingPosDeg", -80);
	
	private static final double GEAR_RATIO = 75.0;
	private static final MomentOfInertia MOI = KilogramSquareMeters.of(.003);
	private static final int LEADER_ID = 14;
	private static final int FOLLOWER_ID = 15;
	private static final DCMotor MOTOR_KIND = DCMotor.getFalcon500(1);
	private static final SparkBaseConfig CONFIG =
		new SparkMaxConfig()
			.idleMode(IdleMode.kBrake)
			.inverted(true);
	
	@Logged private final Motor motor = new ChargerSpark(LEADER_ID, Model.SPARK_MAX, CONFIG)
		                            .withSim(SimDynamics.of(MOTOR_KIND, GEAR_RATIO, MOI), MOTOR_KIND);
	private final SparkMax follower = new SparkMax(FOLLOWER_ID, kBrushless);
	@Logged private final DigitalInput limitSwitch = new DigitalInput(2);
	@Logged private double voltageReq = 0;
	@Logged private boolean shouldApplyKg = false;
	
	public Climber() {
		resetStartingAngle().schedule();
		motor.setControlsConfig(Motor.ControlsConfig.EMPTY.withGearRatio(GEAR_RATIO));
		tryUntilOk(follower, () -> follower.configure(CONFIG.follow(LEADER_ID, true), kResetSafeParameters, kPersistParameters));
	}
	
	public Command setPowerCmd(InputStream output) {
		return this.run(() -> {
			voltageReq = output.get() * 12;
			if (limitSwitch.get() && voltageReq < 0) {
				requestStop();
			} else {
				voltageReq += gravityCompensationV();
				motor.setVoltage(voltageReq);
			}
		})
	       .withName("set voltage(climber)");
	}
	
	@Logged
	public double gravityCompensationV() {
		double angleRads = motor.encoder().positionRad();
		shouldApplyKg = angleRads > degreesToRadians(KG_START_LIMIT_DEG.get());
		return shouldApplyKg ? KG_VOLTS.get() * Math.cos(angleRads) : 0;
	}
	
	public Command resetStartingAngle() {
		return Commands.runOnce(() -> motor.encoder().setPositionReading(Degrees.of(STARTING_POS_DEG.get())))
			       .ignoringDisable(true)
			       .withName("reset climber angle(debug)");
	}
	
	@Override
	protected void requestStop() {
		motor.setVoltage(gravityCompensationV());
	}
}
