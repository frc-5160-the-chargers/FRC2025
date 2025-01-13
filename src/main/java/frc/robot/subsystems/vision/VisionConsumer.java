package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.util.List;

/**
 * A base interface that represents a class
 * that can accept vision data(either obstacles from robot detection
 * or apriltag vision results).
 */
public interface VisionConsumer {
	/** Represents a vision measurement from a camera. */
	record PoseObservation(Pose2d visionPose, double timestampSecs, Matrix<N3, N1> stdDevs) {}
	
	default void addPoseObservation(PoseObservation measurement) {}
	default void setPathfindingObstacles(List<Pose2d> positions) {}
}
