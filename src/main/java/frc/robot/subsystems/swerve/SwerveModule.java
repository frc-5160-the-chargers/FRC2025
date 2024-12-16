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
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import java.util.Optional;

import static edu.wpi.first.epilogue.Logged.Strategy.OPT_IN;
import static edu.wpi.first.units.Units.*;

@Logged(strategy = OPT_IN)
public class SwerveModule {
	private final Motor driveMotor;
	private final Motor steerMotor;
	private final Distance wheelRadius;
	private final LinearVelocity maxVelocity;
	private final SimpleMotorFeedforward velocityFF;
	private final Optional<SwerveModuleSimulation> mapleSim;
	
	@Logged
	@SuppressWarnings("ALL")
	public static class OfLogged extends SwerveModule {
		// these "redundant" fields are logged by epilogue
		private final Motor driveMotor;
		private final Motor steerMotor;
		private final Encoder absoluteEncoder;
		
		public OfLogged(SwerveDriveConfig config, Encoder absoluteEncoder, Motor steerMotor, Motor driveMotor) {
			super(config, absoluteEncoder, steerMotor, driveMotor, Optional.empty());
			this.driveMotor = driveMotor;
			this.steerMotor = steerMotor;
			this.absoluteEncoder = absoluteEncoder;
		}
	}
	
	public SwerveModule(
		SwerveDriveConfig config,
		Encoder absoluteEncoder,
		Motor steerMotor,
		Motor driveMotor,
		Optional<SwerveModuleSimulation> mapleSim
	) {
		this.wheelRadius = config.ofModules().wheelRadius;
		this.maxVelocity = config.ofHardware().maxVelocity();
		this.velocityFF = config.ofControls().velocityFeedforward();
		this.mapleSim = mapleSim;
		if (mapleSim.isPresent() && steerMotor instanceof SimMotor sm && driveMotor instanceof SimMotor dm) {
			this.steerMotor = mapleSim.get().useSteerMotorController(sm);
			this.driveMotor = mapleSim.get().useSteerMotorController(dm);
		} else {
			this.steerMotor = steerMotor;
			this.driveMotor = driveMotor;
			steerMotor.encoder().setPositionReading(absoluteEncoder.position());
		}
	}
	
	public void setDesiredState(SwerveModuleState state, boolean closedLoop) {
		var currentAngle = Rotation2d.fromRadians(driveMotor.encoder().positionRad());
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
	
	public SwerveModuleState currentState() {
		if (mapleSim.isPresent()) {
			return mapleSim.get().getCurrentState();
		} else {
			return new SwerveModuleState(
				driveMotor.encoder().velocityRadPerSec() * wheelRadius.in(Meters),
				Rotation2d.fromRadians(driveMotor.encoder().positionRad())
			);
		}
	}
	
	public SwerveModulePosition currentPosition() {
		if (mapleSim.isPresent()) {
			return new SwerveModulePosition(
				mapleSim.get().getDriveWheelFinalPosition().in(Radians) * wheelRadius.in(Meters),
				mapleSim.get().getSteerAbsoluteFacing()
			);
		} else {
			return new SwerveModulePosition(
				driveMotor.encoder().positionRad() * wheelRadius.in(Meters),
				Rotation2d.fromRadians(driveMotor.encoder().positionRad())
			);
		}
	}
	
	public void setSteerVoltage(double voltage) { steerMotor.setVoltage(voltage); }
	
	public void setDriveVoltage(double voltage){ driveMotor.setVoltage(voltage); }
}
