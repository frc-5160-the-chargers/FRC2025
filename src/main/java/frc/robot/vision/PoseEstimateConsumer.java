package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Represents a function that accepts a Pose2d, a timestamp, and standard deviations,
 * and updates an internal pose estimator.
 */
@FunctionalInterface
public interface PoseEstimateConsumer {
	void addData(Pose2d estimatedPose, double timestampSecs, Matrix<N3, N1> standardDeviations);
}
