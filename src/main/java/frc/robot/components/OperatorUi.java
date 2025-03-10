package frc.robot.components;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A receiver for data sent from the operator UI.
 * The OperatorUI uses the networktables spec to send this data.
 */
public class OperatorUi {
	private final NetworkTableEntry pathfindTargetEntry =
		NetworkTableInstance.getDefault().getEntry("operatorUi/pathfindTarget");
	private final NetworkTableEntry targetLevelEntry =
		NetworkTableInstance.getDefault().getEntry("operatorUi/targetLevel");
	private final NetworkTableEntry manualOverrideEntry =
		NetworkTableInstance.getDefault().getEntry("operatorUi/manualOverrideEnabled");
	
	private int pathfindTarget = -1;
	private int targetLevel = -1;
	private boolean manualOverrideEnabled = true;
	
	/** Must be called periodically in the robotPeriodic method of the Robot class. */
	public void periodic() {
		pathfindTarget = (int) pathfindTargetEntry.getInteger(-1);
		targetLevel = (int) targetLevelEntry.getInteger(-1);
		manualOverrideEnabled = manualOverrideEntry.getBoolean(true);
	}
	
	/**
	 * A trigger that returns true when the operator ui allows for the use
	 * of the manual override controller.
	 */
	@Logged
	public final Trigger isManualOverride =
		new Trigger(() -> manualOverrideEnabled && DriverStation.isTeleop());
	
	/**
	 * A trigger that returns true when the operator ui requests the specified
	 * scoring level(L1-L4).
	 */
	public Trigger targetLevelIs(int level) {
		return new Trigger(() -> this.targetLevel == level);
	}
	
	/**
	 * A trigger that returns true when the operator ui requests the specified pathfind pose.
	 * @see frc.robot.constants.TargetPoses
	 */
	public Trigger pathfindTargetIs(int id) {
		return new Trigger(() -> this.pathfindTarget == id);
	}
}
