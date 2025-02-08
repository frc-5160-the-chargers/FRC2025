package frc.robot.vision;

import edu.wpi.first.math.geometry.Transform3d;

/**
 * Describes a single-tag pose estimate - this has reduced uncertainty for single tag views compared to GlobalPoseEstimate.
 */
public record SingleTagPoseEstimate(
	int tagId,
	Transform3d cameraToTarget,
	Transform3d robotToCameraPosition
) {}
