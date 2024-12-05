package frc.chargers.utils.swerve;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import lombok.RequiredArgsConstructor;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.ControlRequest;

import static edu.wpi.first.epilogue.Logged.Strategy.OPT_IN;
import static edu.wpi.first.units.Units.*;

@Logged(strategy = OPT_IN)
@RequiredArgsConstructor
public class SimSwerveModule implements BaseSwerveModule {
	private final SwerveModuleSimulation mapleSim;
	private final Distance wheelRadius;
	private final LinearVelocity maxVelocity;
	
	@Override
	public void setDesiredState(SwerveModuleState state, boolean closedLoop) {
		state.optimize(mapleSim.getSteerAbsoluteFacing());
		state.cosineScale(mapleSim.getSteerAbsoluteFacing());
		
		mapleSim.requestSteerControl(
			new ControlRequest.PositionVoltage(state.angle.getMeasure())
		);
		
		if (closedLoop) {
			var velocitySetpoint = RadiansPerSecond.of(
				state.speedMetersPerSecond / wheelRadius.in(Meters)
			);
			mapleSim.requestDriveControl(new ControlRequest.VelocityVoltage(velocitySetpoint));
		} else {
			mapleSim.requestDriveControl(
				new ControlRequest.VoltageOut(Volts.of(
					state.speedMetersPerSecond / maxVelocity.in(MetersPerSecond) * 12.0
				))
			);
		}
	}
	
	@Override
	public SwerveModuleState currentState() { return mapleSim.getCurrentState(); }
	
	@Override
	public SwerveModulePosition currentPosition() {
		return new SwerveModulePosition(
			mapleSim.getDriveWheelFinalPosition().in(Radians) * wheelRadius.in(Meters),
			mapleSim.getSteerAbsoluteFacing()
		);
	}
	
	@Override
	public void setSteerVoltage(double voltage) {
		mapleSim.requestSteerControl(
			new ControlRequest.VoltageOut(Volts.of(voltage))
		);
	}
	
	@Override
	public void setDriveVoltage(double voltage) {
		mapleSim.requestDriveControl(
			new ControlRequest.VoltageOut(Volts.of(voltage))
		);
	}
	
	@Override
	public void periodic() {
	}
}
