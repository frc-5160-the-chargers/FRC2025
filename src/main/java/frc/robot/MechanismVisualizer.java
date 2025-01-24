package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import lombok.RequiredArgsConstructor;

import java.util.function.DoubleSupplier;

@RequiredArgsConstructor
public class MechanismVisualizer {
	private final DoubleSupplier elevatorPositionSupplierMeters;
	private final DoubleSupplier pivotAngleSupplierRad;
	
	@Logged private final Pose3d frameRelativePosition = new Pose3d(); // TODO
	
	@Logged
	public Pose3d stage1RelativePosition() {
		var initialPose = new Pose3d(0.14, 0, 0.169, Rotation3d.kZero);
		var elevatorPosition = elevatorPositionSupplierMeters.getAsDouble();
		if (elevatorPosition > 0.706) {
			return initialPose.plus(
				new Transform3d(0, 0, elevatorPosition - 0.706, Rotation3d.kZero)
			);
		} else {
			return initialPose;
		}
	}
	
	@Logged
	public Pose3d carriageRelativePosition() {
		return new Pose3d(
			0.14, 0,
			0.247 + elevatorPositionSupplierMeters.getAsDouble(),
			Rotation3d.kZero
		);
	}
}

