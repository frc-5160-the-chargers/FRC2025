package frc.robot.subsystems.swerve;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.chargers.hardware.encoders.Encoder;
import frc.chargers.hardware.motorcontrol.Motor;
import frc.chargers.hardware.motorcontrol.SimMotor;
import frc.robot.subsystems.swerve.SwerveDrive.SwerveDriveConfig;
import monologue.LogLocal;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

import java.util.Optional;

import static edu.wpi.first.math.MathUtil.angleModulus;
import static edu.wpi.first.units.Units.*;

public class SwerveModule implements LogLocal, AutoCloseable {
	@Logged private final Motor driveMotor;
	@Logged private final Motor steerMotor;
	@Logged private final Encoder absoluteEncoder;
	private final Distance wheelRadius;
	private final LinearVelocity maxVelocity;
	private final SimpleMotorFeedforward velocityFF;
	
	public SwerveModule(
		SwerveDriveConfig config,
		Encoder absoluteEncoder,
		Motor steerMotor,
		Motor driveMotor,
		Optional<SwerveModuleSimulation> mapleSim
	) {
		this(
			config.ofModules().wheelRadius,
			config.ofHardware().maxVelocity(),
			config.ofControls().velocityFeedforward(),
			absoluteEncoder, steerMotor, driveMotor, mapleSim
		);
	}
	
	public SwerveModule(
		Distance wheelRadius,
		LinearVelocity maxVelocity,
		SimpleMotorFeedforward velocityFF,
		Encoder absoluteEncoder,
		Motor steerMotor,
		Motor driveMotor,
		Optional<SwerveModuleSimulation> mapleSim
	) {
		this.wheelRadius = wheelRadius;
		this.maxVelocity = maxVelocity;
		this.velocityFF = velocityFF;
		this.steerMotor = steerMotor;
		this.driveMotor = driveMotor;
		this.absoluteEncoder = absoluteEncoder;
		if (mapleSim.isPresent() && steerMotor instanceof SimMotor sm && driveMotor instanceof SimMotor dm) {
			mapleSim.get().useSteerMotorController(sm.getMapleSimApi());
			mapleSim.get().useDriveMotorController(dm.getMapleSimApi());
		} else {
			steerMotor.encoder().setPositionReading(absoluteEncoder.position());
			driveMotor.encoder().setPositionReading(Degrees.zero());
		}
	}
	
	public void setDesiredState(SwerveModuleState state, boolean closedLoop) {
		var currentAngle = Rotation2d.fromRadians(angleModulus(steerMotor.encoder().positionRad()));
		state.optimize(currentAngle);
		state.cosineScale(currentAngle);
		
		steerMotor.moveToPosition(state.angle.getRadians());
		if (closedLoop) {
			var speedSetpoint = state.speedMetersPerSecond / wheelRadius.in(Meters);
			driveMotor.setVelocity(speedSetpoint, velocityFF.calculate(speedSetpoint));
		} else {
			var voltage = state.speedMetersPerSecond / maxVelocity.in(MetersPerSecond) * 12.0;
			driveMotor.setVoltage(voltage);
		}
	}
	
	public void setAngle(double radians) {
		steerMotor.moveToPosition(radians);
	}
	
	public SwerveModuleState currentState() {
		return new SwerveModuleState(
			driveMotor.encoder().velocityRadPerSec() * wheelRadius.in(Meters),
			Rotation2d.fromRadians(steerMotor.encoder().positionRad())
		);
	}
	
	public SwerveModulePosition currentPosition() {
		return new SwerveModulePosition(
			driveMotor.encoder().positionRad() * wheelRadius.in(Meters),
			Rotation2d.fromRadians(steerMotor.encoder().positionRad())
		);
	}
	
	public void setSteerVoltage(double voltage) {
		steerMotor.setVoltage(voltage);
	}
	
	public void setDriveVoltage(double voltage){
		driveMotor.setVoltage(voltage);
	}
	
	@Override
	public void close() throws Exception {
		driveMotor.close();
		steerMotor.close();
	}
}
