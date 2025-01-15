package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import lombok.Setter;
import monologue.LogLocal;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.wpilibj.Alert.AlertType.kError;
import static frc.chargers.utils.UtilMethods.toIntArray;

/**
 * Important: std dev = standard deviation
 */
@SuppressWarnings("unused")
public class AprilTagVision implements AutoCloseable, LogLocal {
	private static final double MAX_SINGLE_TAG_AMBIGUITY = 0.3;
	private static final Distance MAX_Z_ERROR = Meters.of(0.75);
	private static final double LINEAR_STD_DEV_BASELINE = 0.02;
	private static final double ANGULAR_STD_DEV_BASELINE = 0.06;
	private static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
	private static final List<PhotonCamConfig> PHOTON_TAG_CAMERAS = List.of(
		new PhotonCamConfig("camera_0", 1.0, new Transform3d())
			.withSim(new SimCameraProperties())
	);
	private static final VisionSystemSim VISION_SYSTEM_SIM = new VisionSystemSim("main");
	
	private static class PhotonCamConfig {
		public final PhotonCamera photonCam;
		public final double stdDevFactor;
		public final PhotonPoseEstimator poseEstimator;
		
		public PhotonCamConfig(String cameraName, double stdDevFactor, Transform3d robotToCamera) {
			this.photonCam = new PhotonCamera(cameraName);
			this.stdDevFactor = stdDevFactor;
			this.poseEstimator = new PhotonPoseEstimator(FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
		}
		
		public PhotonCamConfig withSim(SimCameraProperties props) {
			VISION_SYSTEM_SIM.addCamera(new PhotonCameraSim(photonCam, props), poseEstimator.getRobotToCameraTransform());
			return this;
		}
	}
	
	// defaults to empty vision consumer
	@Setter private VisionConsumer visionConsumer = new VisionConsumer() {};
	@Setter private Supplier<Pose2d> simPoseSupplier = () -> null;
	private final Alert connectionAlert = new Alert("", kError);
	@Logged private final List<Pose3d> acceptedPoses = new ArrayList<>();
	@Logged private final List<Pose3d> rejectedPoses = new ArrayList<>();
	private final Set<Integer> fiducialIds = new HashSet<>();
	
	public AprilTagVision(TimedRobot robot) {
		robot.addPeriodic(this::periodic, 0.02);
		if (RobotBase.isSimulation()) VISION_SYSTEM_SIM.addAprilTags(FIELD_LAYOUT);
	}
	
	private void periodic() {
		acceptedPoses.clear();
		rejectedPoses.clear();
		fiducialIds.clear();
		
		var simPose = simPoseSupplier.get();
		if (simPose != null) VISION_SYSTEM_SIM.update(simPose);
		
		var disconnectedCamNames = new ArrayList<String>();
		for (var cam: PHOTON_TAG_CAMERAS) {
			if (!cam.photonCam.isConnected()) {
				disconnectedCamNames.add(cam.photonCam.getName());
				continue;
			}
			
			for (var camData: cam.photonCam.getAllUnreadResults()) {
				boolean ambiguityExceeded = camData.targets.size() == 1 && camData.targets.get(0).poseAmbiguity > MAX_SINGLE_TAG_AMBIGUITY;
				if (!camData.hasTargets() || ambiguityExceeded) continue;
				fiducialIds.addAll(camData.targets.stream().map(it -> it.fiducialId).toList());
				
				var result = cam.poseEstimator.update(camData);
				if (result.isEmpty()) continue;
				
				var pose = result.get().estimatedPose;
				if (Math.abs(pose.getZ()) > MAX_Z_ERROR.in(Meters) // Must have realistic Z coordinate
					    // Must be within the field boundaries
					    || pose.getX() < 0.0
					    || pose.getX() > FIELD_LAYOUT.getFieldLength()
					    || pose.getY() < 0.0
					    || pose.getY() > FIELD_LAYOUT.getFieldWidth()) {
					rejectedPoses.add(pose);
					continue;
				}
				acceptedPoses.add(pose);
				double tagDistSum = 0.0;
				for (var target: camData.targets) {
					tagDistSum += target.bestCameraToTarget.getTranslation().getNorm();
				}
				double stdDevMultiplier = Math.pow(tagDistSum / camData.targets.size(), 2) / camData.targets.size();
				double linearStdDev = stdDevMultiplier * LINEAR_STD_DEV_BASELINE * cam.stdDevFactor;
				double angularStdDev = stdDevMultiplier * ANGULAR_STD_DEV_BASELINE * cam.stdDevFactor;
				
				visionConsumer.addVisionPoseEstimate(
					pose.toPose2d(),
					result.get().timestampSeconds,
					VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)
				);
			}
		}
		
		connectionAlert.setText("The following cameras are disconnected: " + disconnectedCamNames);
		connectionAlert.set(!disconnectedCamNames.isEmpty());
		log("disconnectedCameras", disconnectedCamNames.toArray(new String[0]));
		log("fiducialIds", toIntArray(fiducialIds));
	}
	
	@Override
	public void close() {
		for (var cam: PHOTON_TAG_CAMERAS) {
			cam.photonCam.close();
		}
	}
}
