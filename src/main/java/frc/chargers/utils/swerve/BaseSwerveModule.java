package frc.chargers.utils.swerve;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static edu.wpi.first.epilogue.Logged.Strategy.OPT_IN;

@Logged(strategy = OPT_IN)
public interface BaseSwerveModule {
	void setDesiredState(SwerveModuleState state, boolean closedLoop);
	SwerveModuleState currentState();
	SwerveModulePosition currentPosition();
	
	void setSteerVoltage(double voltage);
	void setDriveVoltage(double voltage);
	
	default void periodic(){}
}
