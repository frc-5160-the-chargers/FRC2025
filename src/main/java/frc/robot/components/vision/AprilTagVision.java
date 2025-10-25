package frc.robot.components.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.chargers.data.LoggingUtil;
import frc.chargers.data.RobotMode;
import frc.chargers.misc.Tracer;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import java.util.*;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static frc.robot.components.vision.VisionConsts.*;

public class AprilTagVision {
    /** A pose estimate. */
    public record PoseEstimate(
        Pose2d pose,
        double timestampSecs,
        Vector<N3> stdDevs
    ) {}

    private final Map<Camera, PhotonPoseEstimator> poseEstimators = new HashMap<>();
    private final List<PoseEstimate> poseEstimates = new ArrayList<>();
    private final Supplier<Pose2d> simPoseSupplier;

    public AprilTagVision(Supplier<Pose2d> simPoseSupplier) {
        this.simPoseSupplier = simPoseSupplier;
        addCamera(REEF_TAGS, new Camera("Chargers-FrontRight", 1, FR_CAM_TRANSFORM));
        addCamera(REEF_TAGS, new Camera("Chargers-FrontLeft", 1, FL_CAM_TRANSFORM));
    }

    private void addCamera(AprilTagFieldLayout tags, Camera camera) {
        var est = new PhotonPoseEstimator(
            tags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera.robotToCamera
        );
        est.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        poseEstimators.put(camera, est);
    }

    public List<PoseEstimate> update(Rotation2d heading, double headingTimestampSecs) {
        var strat = DriverStation.isDisabled()
            ? PoseStrategy.LOWEST_AMBIGUITY
            : PoseStrategy.PNP_DISTANCE_TRIG_SOLVE;
        for (var est: poseEstimators.values()) {
            est.setMultiTagFallbackStrategy(strat);
            est.addHeadingData(headingTimestampSecs, heading);
        }
        return update();
    }

    public List<PoseEstimate> update() {
        poseEstimates.clear();
        VISION_SYSTEM_SIM.ifPresent(s -> s.update(simPoseSupplier.get()));
        Tracer.startTrace("compute pose estimates");
        for (var cam: poseEstimators.keySet()) {
            cam.refreshData();
            Tracer.trace(
                "Process Inputs(vision)",
                () -> Logger.processInputs(cam.logKey(""), cam.inputs)
            );

            if (RobotMode.get() == RobotMode.REAL && !cam.inputs.connected) {
                continue;
            }
            var poseEstimator = poseEstimators.get(cam);
            int ambHighCount = 0;
            int errHighCount = 0;
            var fiducialIds = new HashSet<Integer>();
            var poses = new ArrayList<Pose3d>();
            for (var result: cam.inputs.results) {
                // ignores result if ambiguity is exceeded or if there is no targets.
                if (result.targets.isEmpty()) {
                    continue;
                }
                boolean ambiguityExceeded = true;
                // Computes the standard deviations of the pose pose,
                // scaling off distance from the target, z error, and # of targets.
                double tagDistSum = 0.0;
                double tagAreaSum = 0.0;
                for (var target: result.targets) {
                    ambiguityExceeded = ambiguityExceeded && target.poseAmbiguity > MAX_AMBIGUITY;
                    tagDistSum += target.bestCameraToTarget.getTranslation().getNorm();
                    tagAreaSum += target.area;
                    fiducialIds.add(target.fiducialId);
                }
                if (ambiguityExceeded) {
                    ambHighCount++;
                    continue;
                }

                // updates the pose pose, and makes sure that the estimated pose
                // has a z coordinate near 0 and x and y coordinates within the field.
                var poseEstimate = poseEstimator.update(result);
                if (poseEstimate.isEmpty()) continue;
                var pose = poseEstimate.get().estimatedPose;
                var timestamp = poseEstimate.get().timestampSeconds;
                if (Math.abs(pose.getZ()) > MAX_Z_ERROR.in(Meters)
                    || pose.getX() < 0.0
                    || pose.getX() > ALL_TAGS.getFieldLength()
                    || pose.getY() < 0.0
                    || pose.getY() > ALL_TAGS.getFieldWidth()) {
                    errHighCount++;
                    continue;
                }

                // Calculates standard deviations
                double areaSumMultiplier = Math.pow(result.targets.size() / Math.abs(tagAreaSum), 0.2);
                double stdDevMultiplier = Math.pow(tagDistSum / result.targets.size(), 2) / result.targets.size();
                stdDevMultiplier *= Math.pow(Z_ERROR_SCALAR, Math.abs(pose.getZ()));
                stdDevMultiplier *= Math.max(areaSumMultiplier, 1);
                if (result.targets.size() <= 1) stdDevMultiplier *= SINGLE_TAG_SCALAR;
                double linearStdDev = stdDevMultiplier * LINEAR_STD_DEV_BASELINE * cam.stdDevFactor;

                // Registers the pose pose.
                poses.add(pose);
                var stdDevs = VecBuilder.fill(linearStdDev, linearStdDev, ANGULAR_STD_DEV);
                poseEstimates.add(new PoseEstimate(pose.toPose2d(), timestamp, stdDevs));
            }

            // logs relevant data
            LoggingUtil.logIntList(cam.logKey("fiducialIds"), fiducialIds);
            Logger.recordOutput(cam.logKey("numAmbiguityExceeded"), ambHighCount);
            Logger.recordOutput(cam.logKey("numErrExceeded"), errHighCount);
            Logger.recordOutput(cam.logKey("poses"), poses.toArray(new Pose3d[0]));
        }
        Tracer.endTrace();
        return poseEstimates;
    }
}