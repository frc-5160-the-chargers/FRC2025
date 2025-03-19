package frc.robot.components.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
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
import frc.robot.subsystems.swerve.SwerveConfigurator;
import lombok.Getter;
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
import java.util.Optional;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj.Alert.AlertType.kError;
import static frc.chargers.utils.UtilMethods.toDoubleArray;
import static frc.chargers.utils.UtilMethods.toIntArray;

@SuppressWarnings("unused")
public class AprilTagVision implements AutoCloseable, LogLocal {
	private static final Optional<VisionSystemSim> VISION_SYSTEM_SIM =
		RobotBase.isSimulation() ? Optional.of(new VisionSystemSim("main")) : Optional.empty();
	private static final SimCameraProperties ARDUCAM_SIM_PROPERTIES = new SimCameraProperties();
	private static final double MAX_SINGLE_TAG_AMBIGUITY = 0.1;
	private static final Distance MAX_Z_ERROR = Meters.of(0.1);
	private static final double Z_ERROR_SCALAR = 100.0;
	private static final double LINEAR_STD_DEV_BASELINE = 0.2;
	private static final double ANGULAR_STD_DEV_BASELINE = 90000;
	
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
	
	/*
	What 254 does for vision filtering:
	1. Area filtering - must be > 0.4 for enabled
	2.
	 */
	
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
	
	@Setter private Consumer<PoseEstimate> globalEstimateConsumer = estimate -> {};
	@Setter private Supplier<Pose2d> simPoseSupplier = null;
	
	private final Alert connectionAlert = new Alert("", kError);
	@Logged private final List<Pose3d> acceptedPoses = new ArrayList<>();
	@Logged private final List<Pose3d> rejectedPoses = new ArrayList<>();
	private final List<Double> timestamps = new ArrayList<>();
	@Getter private final Set<Integer> fiducialIds = new HashSet<>();
	private final List<String> disconnectedCamNames = new ArrayList<>();
	
	/** Must be called periodically in the robotPeriodic method of the Robot class. */
	public void periodic() {
		acceptedPoses.clear();
		rejectedPoses.clear();
		fiducialIds.clear();
		timestamps.clear();
		disconnectedCamNames.clear();
		if (VISION_SYSTEM_SIM.isPresent() && simPoseSupplier != null) {
			VISION_SYSTEM_SIM.get().update(simPoseSupplier.get());
		}
		for (var config: PHOTON_CAM_CONFIGS) {
			if (!RobotBase.isSimulation() && !config.photonCam.isConnected()) {
				disconnectedCamNames.add(config.photonCam.getName());
				continue;
			}
			for (var camData: config.photonCam.getAllUnreadResults()) {
				config.poseEstimator.setPrimaryStrategy(
					DriverStation.isDisabled() ? PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR : PoseStrategy.PNP_DISTANCE_TRIG_SOLVE
				);
				boolean ambiguityExceeded = camData.targets.size() == 1 && camData.targets.get(0).poseAmbiguity > MAX_SINGLE_TAG_AMBIGUITY;
				if (!camData.hasTargets() || ambiguityExceeded) continue;
				fiducialIds.addAll(camData.targets.stream().map(it -> it.fiducialId).toList());
				var result = config.poseEstimator.update(camData);
				if (result.isEmpty()) continue;
				var pose = result.get().estimatedPose;
				if (Math.abs(pose.getZ()) > MAX_Z_ERROR.in(Meters) // Must have realistic Z coordinate
				    // Must be within the field boundaries
				    || pose.getX() < 0.0
				    || pose.getX() > FIELD_LAYOUT.getFieldLength()
				    || pose.getY() < 0.0
				    || pose.getY() > FIELD_LAYOUT.getFieldWidth()
					|| camData.targets.isEmpty()) {
					rejectedPoses.add(pose);
					continue;
				}
				acceptedPoses.add(pose);
				double tagDistSum = 0.0;
				for (var target: camData.targets) {
					tagDistSum += target.bestCameraToTarget.getTranslation().getNorm();
				}
				double stdDevMultiplier = Math.pow(tagDistSum / camData.targets.size(), 2) / camData.targets.size();
				stdDevMultiplier *= Math.pow(Z_ERROR_SCALAR, Math.abs(pose.getZ()));
				double linearStdDev = stdDevMultiplier * LINEAR_STD_DEV_BASELINE * config.stdDevFactor;
				log("linearStdDevs/" + config.photonCam.getName() + "/linear", linearStdDev);
				if (Double.isNaN(linearStdDev)) {
					System.out.println("LMAO");
				}
				
				timestamps.add(result.get().timestampSeconds);
				globalEstimateConsumer.accept(
					new PoseEstimate(
						pose.toPose2d(),
						result.get().timestampSeconds,
						VecBuilder.fill(linearStdDev, linearStdDev, 100000000)
					)
				);
			}
		}
		connectionAlert.setText("The following cameras are disconnected: " + disconnectedCamNames);
		connectionAlert.set(!disconnectedCamNames.isEmpty());
		log("disconnectedCameras", disconnectedCamNames.toArray(new String[0]));
		log("timestamps", toDoubleArray(timestamps));
		log("fiducialIds", toIntArray(fiducialIds));
	}
	
	@Override
	public void close() {
		for (var cam: PHOTON_CAM_CONFIGS) {
			cam.photonCam.close();
		}
	}
}
