package frc.robot.field;

import edu.wpi.first.math.geometry.Pose2d;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

public class ScoringPoses {
	private static final Map<Integer, Pose2d> NODE_POSITION_MAP = new HashMap<>();
	
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
	}
	
	public static Optional<Pose2d> getPose(int nodePosition) {
		return Optional.ofNullable(NODE_POSITION_MAP.get(nodePosition));
	}
}
