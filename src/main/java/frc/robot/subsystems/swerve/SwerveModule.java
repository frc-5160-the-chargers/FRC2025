package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import frc.chargers.hardware.encoders.Encoder;
import frc.chargers.hardware.motorcontrol.ChargerTalonFX;
import frc.chargers.hardware.motorcontrol.Motor;
import frc.chargers.hardware.motorcontrol.SimMotor;
import frc.robot.subsystems.swerve.SwerveDrive.SwerveDriveConfig;
import lombok.RequiredArgsConstructor;
import monologue.LogLocal;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.jetbrains.annotations.Nullable;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

// creates a constructor w/ all variables marked final(i.e driveMotor, steerMotor, etc.)
@RequiredArgsConstructor
public class SwerveModule implements LogLocal, AutoCloseable {
	@Logged private final Motor driveMotor;
	@Logged private final Motor steerMotor;
	@Logged private final Encoder absoluteEncoder;
	private final Distance wheelRadius;
	private final LinearVelocity maxVelocity;
	private final SimpleMotorFeedforward velocityFF;
	
	private static final Alert FF_ALERT = new Alert(
		"Additional FF is > 0 in open loop drive mode",
		Alert.AlertType.kWarning
	);
	
	/**
	 * A dummy motor that accepts MapleSim data via MapleSim's SimulatedMotorController,
	 * propagating it through a TalonFX.
	 */
	public static class DummyMotor extends ChargerTalonFX implements SimulatedMotorController {
		private final TalonFXSimState talonSimApi;
		
		public DummyMotor(@Nullable TalonFXConfiguration config) {
			super(SimMotor.getDummyId(), false, config);
			talonSimApi = super.baseApi.getSimState();
		}
		
		@Override
		public Voltage updateControlSignal(
			Angle angle, AngularVelocity angularVelocity,
			Angle encoderAngle, AngularVelocity encoderVelocity
		) {
			talonSimApi.setSupplyVoltage(12.0);
			talonSimApi.setRawRotorPosition(encoderAngle);
			talonSimApi.setRotorVelocity(encoderVelocity);
			return talonSimApi.getMotorVoltageMeasure();
		}
	}
	
	public SwerveModule(
		Motor driveMotor,
		Motor steerMotor,
		Encoder absoluteEncoder,
		SwerveDriveConfig config
	) {
		this(
			driveMotor, steerMotor, absoluteEncoder,
			config.ofModules().wheelRadius,
			config.ofHardware().maxVelocity(),
			config.ofControls().velocityFeedforward()
		);
	}
	
	public void setDesiredState(SwerveModuleState state, boolean closedLoop, double additionalFeedforward) {
		steerMotor.moveToPosition(state.angle.getRadians());
		if (closedLoop) {
			var speedSetpoint = state.speedMetersPerSecond / wheelRadius.in(Meters);
			driveMotor.setVelocity(speedSetpoint, velocityFF.calculate(speedSetpoint) + additionalFeedforward);
		} else {
			FF_ALERT.set(additionalFeedforward > 0.0);
			var voltage = state.speedMetersPerSecond / maxVelocity.in(MetersPerSecond) * 12.0;
			driveMotor.setVoltage(voltage);
		}
	}
	
	public Rotation2d getSteerAngle() {
		return Rotation2d.fromRadians(steerMotor.encoder().positionRad());
	}
	
	public SwerveModuleState currentState() {
		return new SwerveModuleState(
			driveMotor.encoder().velocityRadPerSec() * wheelRadius.in(Meters),
			getSteerAngle()
		);
	}
	
	public SwerveModulePosition currentPosition() {
		return new SwerveModulePosition(
			driveMotor.encoder().positionRad() * wheelRadius.in(Meters),
			getSteerAngle()
		);
	}
	
	public void setSteerVoltage(double voltage) {
		steerMotor.setVoltage(voltage);
	}
	
	public void setDriveVoltage(double voltage){
		driveMotor.setVoltage(voltage);
	}
	
	@Override
	public void close() {
		driveMotor.close();
		steerMotor.close();
		absoluteEncoder.close();
	}
}
