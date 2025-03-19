package frc.robot.components.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** A recorded pose estimate from photonvision. */
public record PoseEstimate(
	Pose2d pose,
	double timestampSecs,
	Matrix<N3, N1> standardDeviations
) {}
