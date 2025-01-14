package frc.robot.targeting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import java.util.HashMap;
import java.util.Map;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

public record ScoringPosition(int scoringLevel, int nodePosition) {
	private static final Map<Integer, Pose2d> NODE_POSITION_MAP = new HashMap<>();
	private static final Map<Integer, Distance> LEVEL_TO_ELEVATOR_HEIGHT_MAP = new HashMap<>();
	private static final Map<Integer, Angle> LEVEL_TO_PIVOT_ANGLE_MAP = new HashMap<>();
	
	static {
		NODE_POSITION_MAP.put(1, Pose2d.kZero);
		NODE_POSITION_MAP.put(2, Pose2d.kZero);
		NODE_POSITION_MAP.put(3, Pose2d.kZero);
		NODE_POSITION_MAP.put(4, Pose2d.kZero);
		NODE_POSITION_MAP.put(5, Pose2d.kZero);
		NODE_POSITION_MAP.put(6, Pose2d.kZero);
		NODE_POSITION_MAP.put(7, Pose2d.kZero);
		NODE_POSITION_MAP.put(8, Pose2d.kZero);
		NODE_POSITION_MAP.put(9, Pose2d.kZero);
		NODE_POSITION_MAP.put(10, Pose2d.kZero);
		NODE_POSITION_MAP.put(11, Pose2d.kZero);
		NODE_POSITION_MAP.put(12, Pose2d.kZero);
		
		LEVEL_TO_ELEVATOR_HEIGHT_MAP.put(1, Meters.of(0));
		LEVEL_TO_ELEVATOR_HEIGHT_MAP.put(2, Meters.of(1));
		LEVEL_TO_ELEVATOR_HEIGHT_MAP.put(3, Meters.of(2));
		LEVEL_TO_ELEVATOR_HEIGHT_MAP.put(4, Meters.of(3));
		
		LEVEL_TO_PIVOT_ANGLE_MAP.put(1, Radians.of(0));
		LEVEL_TO_PIVOT_ANGLE_MAP.put(2, Radians.of(1));
		LEVEL_TO_PIVOT_ANGLE_MAP.put(3, Radians.of(2));
		LEVEL_TO_PIVOT_ANGLE_MAP.put(4, Radians.of(3));
	}
	
	/**
	 * Creates a new ScoringPosition.
	 * @param scoringLevel The target height. Can be L1, L2, L3, or L4.
	 * @param nodePosition The target position. From a birds eye view,
	 *                     position 1 is the topmost position of the reef hexagon face
	 *                     that directly faces the centerline/net. All other positions
	 *                     are numbered in a clockwise manner. Can be -1 if the scoringLevel is 1.
	 */
	public ScoringPosition(int scoringLevel, int nodePosition) {
		this.nodePosition = nodePosition;
		this.scoringLevel = scoringLevel;
		if ((nodePosition < 0 || nodePosition > 12) && scoringLevel != 1) {
			throw new RuntimeException("Invalid node position: " + nodePosition);
		}
		if (scoringLevel < 1 || scoringLevel > 4) {
			throw new RuntimeException("Invalid scoring level: " + scoringLevel);
		}
	}
	
	public Pose2d pathfindingTarget() {
		return NODE_POSITION_MAP.get(nodePosition);
	}
	
	public Distance elevatorHeight() {
		return LEVEL_TO_ELEVATOR_HEIGHT_MAP.get(scoringLevel);
	}
	
	public Angle pivotAngle() {
		return LEVEL_TO_PIVOT_ANGLE_MAP.get(scoringLevel);
	}
}
