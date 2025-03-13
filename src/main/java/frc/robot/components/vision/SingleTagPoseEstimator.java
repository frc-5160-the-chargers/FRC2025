package frc.robot.components.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.chargers.utils.data.TunableValues.TunableNum;
import lombok.Getter;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import static monologue.Monologue.GlobalLog;

/**
 * An implementation of 6328's trig-based pose estimation algorithm.
 * This algorithm provides a significant decrease in estimation uncertainty,
 * at the cost of a slight increase in inaccuracy(as well as focusing only on 1 tag at a time).
 * TODO: This is not working yet...
 */
public class SingleTagPoseEstimator {
	// Must be less than 2.0
	private static final TunableNum txTyObservationStaleSecs =
		new TunableNum("RobotState/TxTyObservationStaleSeconds", 0.5);
	
	private static final double poseBufferSizeSec = 2.0;
	private static final Map<Integer, Pose2d> tagPoses2d = new HashMap<>();
	
	static {
		var tags = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTags();
		for (int i = 0; i < tags.size(); i++) {
			tagPoses2d.put(i+1, tags.get(i).pose.toPose2d());
		}
	}
	
	// Pose Estimation Members
	@Getter private Pose2d odometryPose = Pose2d.kZero;
	
	private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
		TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);
	// Odometry
	private final SwerveDriveKinematics kinematics;
	private SwerveModulePosition[] lastWheelPositions =
		new SwerveModulePosition[] {
			new SwerveModulePosition(),
			new SwerveModulePosition(),
			new SwerveModulePosition(),
			new SwerveModulePosition()
		};
	// Assume gyro starts at zero
	private Rotation2d gyroOffset = Rotation2d.kZero;
	private final Map<Integer, TxTyPoseRecord> txTyPoses = new HashMap<>();
	
	private record TxTyPoseRecord(Pose2d pose, double timestamp) {}
	
	public SingleTagPoseEstimator(SwerveDriveKinematics kinematics, Matrix<N3, N1> odometryStateStdDevs) {
		this.kinematics = kinematics;
		for (int i = 1; i <= AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTags().size(); i++) {
			txTyPoses.put(i, new TxTyPoseRecord(Pose2d.kZero, -1.0));
		}
	}
	
	public void resetPose(Pose2d pose, Pose2d estimatedPose) {
		// Gyro offset is the rotation that maps the old gyro rotation (estimated - offset) to the new
		// frame of rotation
		gyroOffset = pose.getRotation().minus(estimatedPose.getRotation().minus(gyroOffset));
		odometryPose = pose;
		poseBuffer.clear();
	}
	
	public void update(Rotation2d gyroAngle, SwerveModulePosition... modulePositions) {
		Twist2d twist = kinematics.toTwist2d(lastWheelPositions, modulePositions);
		lastWheelPositions = modulePositions;
		odometryPose = odometryPose.exp(twist);
		odometryPose = new Pose2d(odometryPose.getTranslation(), gyroAngle.plus(gyroOffset));
		// Add pose to buffer at timestamp
		poseBuffer.addSample(Timer.getTimestamp(), odometryPose);
	}
	
	/**
	 * Adds a vision measurement from a single tag.
	 */
	public void addVisionMeasurement(SingleTagPoseEstimate estimate, Pose2d rootPose) {
		// Skip if current data for tag is newer
		if (txTyPoses.containsKey(estimate.tagId())
			    && txTyPoses.get(estimate.tagId()).timestamp() >= estimate.timestampSecs()) {
			return;
		}
		
		// Get rotation at timestamp
		var sample = poseBuffer.getSample(estimate.timestampSecs());
		if (sample.isEmpty()) {
			// exit if not there
			return;
		}
		Rotation2d robotRotation =
			rootPose.transformBy(new Transform2d(odometryPose, sample.get())).getRotation();
		
		Rotation2d camToTagRotation =
			robotRotation.plus(
				estimate.robotToCameraPosition().getRotation().toRotation2d().plus(estimate.cameraToTarget().getRotation().toRotation2d()));
		var tagPose2d = tagPoses2d.get(estimate.tagId());
		if (tagPose2d == null) return;
		Translation2d fieldToCameraTranslation =
			new Pose2d(tagPose2d.getTranslation(), camToTagRotation.plus(Rotation2d.kPi))
				.transformBy(new Transform2d(estimate.cameraToTarget().getTranslation().getNorm(), 0.0, Rotation2d.kZero))
				.getTranslation();
		Pose2d robotPose =
			new Pose2d(
				fieldToCameraTranslation, robotRotation.plus(estimate.robotToCameraPosition().getRotation().toRotation2d()))
				.transformBy(new Transform2d(Pose3d.kZero.transformBy(estimate.robotToCameraPosition()).toPose2d(), Pose2d.kZero));
		// Use gyro angle at time for robot rotation
		robotPose = new Pose2d(robotPose.getTranslation(), robotRotation);
		
		// Add transform to current odometry based pose for latency correction
		txTyPoses.put(
			estimate.tagId(),
			new TxTyPoseRecord(robotPose, estimate.timestampSecs())
		);
	}
	
	/**
	 * Returns an estimated position of the single tag pose estimator.
	 * @param tagIdToFocusOn The tag ID on which to base the pose estimate from.
	 *                       For instance, if you are auto-aligning to a coral position, you
	 *                       would put the tag ID as the closest apriltag to you.
	 * @return A Optional that is empty if no estimated position can be found.
	 */
	public Optional<Pose2d> getEstimatedPosition(int tagIdToFocusOn) {
		if (!txTyPoses.containsKey(tagIdToFocusOn)) {
			DriverStation.reportError("No tag with id: " + tagIdToFocusOn, true);
			return Optional.empty();
		}
		
		var data = txTyPoses.get(tagIdToFocusOn);
		// Check if stale
		if (Timer.getTimestamp() - data.timestamp() >= txTyObservationStaleSecs.get()) {
			GlobalLog.log("stale", true);
			return Optional.empty();
		}
		GlobalLog.log("stale", false);
		// Get odometry based pose at timestamp
		var sample = poseBuffer.getSample(data.timestamp());
		GlobalLog.log("sampleIsEmpty", sample.isEmpty());
		// Latency compensate
		return sample.map(pose2d -> data.pose().plus(new Transform2d(pose2d, odometryPose)));
	}
}
