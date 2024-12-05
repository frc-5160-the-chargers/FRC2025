package frc.robot.subsystems.swerve;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.chargers.hardware.encoders.EncoderIO;
import frc.chargers.hardware.motorcontrol.MotorIO;
import lombok.RequiredArgsConstructor;

import static edu.wpi.first.epilogue.Logged.Strategy.OPT_IN;
import static edu.wpi.first.units.Units.*;
import static frc.chargers.utils.UtilMethods.waitThenRun;

@Logged(strategy = OPT_IN)
@RequiredArgsConstructor
public class RealSwerveModule implements BaseSwerveModule {
	@Logged private final MotorIO driveMotor;
	@Logged private final MotorIO steerMotor;
	@Logged private final EncoderIO absoluteEncoder;
	private final Distance wheelRadius;
	private final LinearVelocity maxSpeed;
	private final SimpleMotorFeedforward velocityFF;
	
	{ waitThenRun(Seconds.of(0.1), this::resetPosition); } // runs when RealSwerveModule is created
	
	private void resetPosition() {
		steerMotor.encoder().setPositionReading(absoluteEncoder.position());
	}
	
	@Override
	public void setDesiredState(SwerveModuleState state, boolean closedLoop) {
		var currentAngle = Rotation2d.fromRadians(driveMotor.encoder().positionRad());
		state.optimize(currentAngle);
		state.cosineScale(currentAngle);
		
		steerMotor.moveToPosition(state.angle.getRadians());
		if (closedLoop) {
			var speedSetpoint = RadiansPerSecond.of(state.speedMetersPerSecond / wheelRadius.in(Meters));
			driveMotor.setVelocity(speedSetpoint, velocityFF.calculate(speedSetpoint).in(Volts));
		} else {
			var voltage = state.speedMetersPerSecond / maxSpeed.in(MetersPerSecond) * 12.0;
			driveMotor.setVoltage(voltage);
		}
	}
	
	@Override
	public SwerveModuleState currentState() {
		return new SwerveModuleState(
			driveMotor.encoder().velocityRadPerSec() * wheelRadius.in(Meters),
			Rotation2d.fromRadians(driveMotor.encoder().positionRad())
		);
	}
	
	@Override
	public SwerveModulePosition currentPosition() {
		return new SwerveModulePosition(
			driveMotor.encoder().positionRad() * wheelRadius.in(Meters),
			Rotation2d.fromRadians(driveMotor.encoder().positionRad())
		);
	}
	
	@Override
	public void setSteerVoltage(double voltage) { steerMotor.setVoltage(voltage); }
	
	@Override
	public void setDriveVoltage(double voltage){ driveMotor.setVoltage(voltage); }
}
