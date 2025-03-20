package frc.robot.components.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CompetitionRobot.SharedState;
import frc.robot.subsystems.swerve.SwerveConfigurator;
import lombok.Setter;
import monologue.LogLocal;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj.Alert.AlertType.kError;
import static frc.chargers.utils.UtilMethods.toIntArray;

@SuppressWarnings("unused")
public class AprilTagVision implements AutoCloseable, LogLocal {
	private static final Optional<VisionSystemSim> VISION_SYSTEM_SIM =
		RobotBase.isSimulation() ? Optional.of(new VisionSystemSim("main")) : Optional.empty();
	private static final SimCameraProperties ARDUCAM_SIM_PROPERTIES = new SimCameraProperties();
	private static final double MAX_SINGLE_TAG_AMBIGUITY = 0.1;
	private static final Distance MAX_Z_ERROR = Meters.of(0.1);
	private static final double Z_ERROR_SCALAR = 100.0;
	private static final double LINEAR_STD_DEV_BASELINE = 0.2; // was 0.5 when last tested - shouldn't be that high prob?
	private static final double ANGULAR_STD_DEV = 10000000;
	
	private static final AprilTagFieldLayout ALL_TAGS_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
	private static final AprilTagFieldLayout REEF_TAGS_ONLY_LAYOUT =
		new AprilTagFieldLayout(
			ALL_TAGS_LAYOUT.getTags()
				.stream()
				.filter(it -> (it.ID >= 17 && it.ID <= 22) || (it.ID >= 6 && it.ID <= 11))
				.toList(),
			ALL_TAGS_LAYOUT.getFieldLength(),
			ALL_TAGS_LAYOUT.getFieldWidth()
		);
	private static final AprilTagFieldLayout FIELD_LAYOUT = REEF_TAGS_ONLY_LAYOUT;
	
	// TODO: Figure out why the robot almost drove into jack
	private static final List<PhotonCamConfig> PHOTON_CAM_CONFIGS = List.of(
		new PhotonCamConfig(
			"Chargers-FrontRight",
			1.0,
			new Transform3d(
				SwerveConfigurator.HARDWARE_SPECS.wheelBase().div(2).minus(Centimeters.of(2.6)),
				SwerveConfigurator.HARDWARE_SPECS.trackWidth().div(-2).plus(Centimeters.of(10.1)),
				Inches.of(7.375),
				new Rotation3d(
					Degrees.zero(),
					Degrees.of(-15),
					Degrees.of(46) // measured as: 46
				)
			)
		).withSim(ARDUCAM_SIM_PROPERTIES),
		new PhotonCamConfig(
			"Chargers-FrontLeft",
			1.0,
			new Transform3d(
				SwerveConfigurator.HARDWARE_SPECS.wheelBase().div(2).minus(Centimeters.of(2.6)),
				SwerveConfigurator.HARDWARE_SPECS.trackWidth().div(2).minus(Centimeters.of(10)),
				Inches.of(7.375),
				new Rotation3d(
					Degrees.zero(),
					Degrees.of(-15),
					Degrees.of(-51) // measured as: -56
				)
			)
		).withSim(ARDUCAM_SIM_PROPERTIES)
	);
	
	// 10.1 cm from right, 7.375 in up, 2.8 cm back
	
	static {
		ARDUCAM_SIM_PROPERTIES.setCalibration(1280, 800, Rotation2d.fromDegrees(70));
		ARDUCAM_SIM_PROPERTIES.setCalibError(0.25, 0.15);
		ARDUCAM_SIM_PROPERTIES.setAvgLatencyMs(35);
		ARDUCAM_SIM_PROPERTIES.setLatencyStdDevMs(5);
		ARDUCAM_SIM_PROPERTIES.setFPS(20);
		VISION_SYSTEM_SIM.ifPresent(it -> it.addAprilTags(FIELD_LAYOUT));
	}
	
	private static class PhotonCamConfig {
		public final PhotonCamera photonCam;
		public final double stdDevFactor;
		public final PhotonPoseEstimator poseEstimator;
		public final Transform3d robotToCamera;
		
		public PhotonCamConfig(String cameraName, double stdDevFactor, Transform3d robotToCamera) {
			this.photonCam = new PhotonCamera(cameraName);
			this.stdDevFactor = stdDevFactor;
			this.poseEstimator = new PhotonPoseEstimator(FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
			this.robotToCamera = robotToCamera;
			poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
		}
		
		public PhotonCamConfig withSim(SimCameraProperties props) {
			VISION_SYSTEM_SIM.ifPresent(
				it -> it.addCamera(new PhotonCameraSim(photonCam, props), poseEstimator.getRobotToCameraTransform())
			);
			return this;
		}
	}
	
	private class CameraStats {
		private final Alert connectionAlert = new Alert("", kError);
		
		public List<Pose3d> acceptedPoses = new ArrayList<>();
		public List<Pose3d> rejectedPoses = new ArrayList<>();
		public Set<Integer> fiducialIds = new HashSet<>();
		public boolean isConnected = true;
		public double linearStdDev = 0.0;
		
		public void reset() {
			acceptedPoses.clear();
			rejectedPoses.clear();
			fiducialIds.clear();
		}
		
		public void logTo(String name) {
			log(name + "/acceptedPoses", acceptedPoses.toArray(new Pose3d[0]));
			log(name + "/rejectedPoses", rejectedPoses.toArray(new Pose3d[0]));
			log(name + "/fiducialIds", toIntArray(fiducialIds));
			log(name + "/isConnected", isConnected);
			log(name + "/linearStdDev", linearStdDev);
			connectionAlert.setText("camera " + name + "is disconnected.");
			connectionAlert.set(isConnected);
		}
	}
	
	@Setter private Consumer<PoseEstimate> globalEstimateConsumer = estimate -> {};
	@Setter private Supplier<Pose2d> simPoseSupplier = null;
	private final Map<PhotonCamConfig, CameraStats> camStatsMap = new HashMap<>();
	private final SharedState sharedState;
	
	public final Trigger hasConnectedCams =
		new Trigger(() -> camStatsMap.values().stream().anyMatch(it -> it.isConnected));
	
	public AprilTagVision(SharedState sharedState) {
		this.sharedState = sharedState;
		for (var config: PHOTON_CAM_CONFIGS) {
			camStatsMap.put(config, new CameraStats());
		}
	}
	
	/** Must be called periodically in the robotPeriodic method of the Robot class. */
	public void periodic() {
		if (VISION_SYSTEM_SIM.isPresent() && simPoseSupplier != null) {
			VISION_SYSTEM_SIM.get().update(simPoseSupplier.get());
		}
		for (var config: PHOTON_CAM_CONFIGS) {
			var cameraStats = camStatsMap.get(config);
			cameraStats.reset();
			if (!RobotBase.isSimulation() && !config.photonCam.isConnected()) {
				cameraStats.isConnected = false;
				cameraStats.logTo(config.photonCam.getName());
				continue;
			}
			config.poseEstimator.setPrimaryStrategy(
				DriverStation.isDisabled() ? PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR : PoseStrategy.PNP_DISTANCE_TRIG_SOLVE
			);
			config.poseEstimator.addHeadingData(sharedState.headingTimestamp.getAsDouble(), sharedState.gyroHeading.get());
			for (var result: config.photonCam.getAllUnreadResults()) {
				// ignores result if ambiguity is exceeded or if there is no targets.
				boolean ambiguityExceeded = result.targets.size() == 1 && result.targets.get(0).poseAmbiguity > MAX_SINGLE_TAG_AMBIGUITY;
				if (!result.hasTargets() || ambiguityExceeded) {
					continue;
				}
				
				// updates the pose estimate, and makes sure that the estimated pose
				// has a z coordinate near 0 and x and y coordinates within the field.
				var poseEstimate = config.poseEstimator.update(result);
				if (poseEstimate.isEmpty()) continue;
				var pose = poseEstimate.get().estimatedPose;
				if (Math.abs(pose.getZ()) > MAX_Z_ERROR.in(Meters)
				    || pose.getX() < 0.0
				    || pose.getX() > FIELD_LAYOUT.getFieldLength()
				    || pose.getY() < 0.0
				    || pose.getY() > FIELD_LAYOUT.getFieldWidth()
					|| result.targets.isEmpty()) {
					cameraStats.rejectedPoses.add(pose);
					continue;
				}
				cameraStats.acceptedPoses.add(pose);
				cameraStats.fiducialIds.addAll(result.targets.stream().map(it -> it.fiducialId).toList());
				
				// Computes the standard deviations of the pose estimate,
				// scaling off distance from the target, z error, and # of targets.
				double tagDistSum = 0.0;
				for (var target: result.targets) {
					tagDistSum += target.bestCameraToTarget.getTranslation().getNorm();
				}
				double stdDevMultiplier = Math.pow(tagDistSum / result.targets.size(), 2) / result.targets.size();
				stdDevMultiplier *= Math.pow(Z_ERROR_SCALAR, Math.abs(pose.getZ()));
				double linearStdDev = stdDevMultiplier * LINEAR_STD_DEV_BASELINE * config.stdDevFactor;
				cameraStats.linearStdDev = linearStdDev;
				
				// Registers the pose estimate.
				globalEstimateConsumer.accept(
					new PoseEstimate(
						pose.toPose2d(),
						poseEstimate.get().timestampSeconds,
						VecBuilder.fill(linearStdDev, linearStdDev, ANGULAR_STD_DEV)
					)
				);
			}
			cameraStats.logTo(config.photonCam.getName());
		}
	}
	
	@Override
	public void close() {
		for (var cam: PHOTON_CAM_CONFIGS) {
			cam.photonCam.close();
		}
	}
}
