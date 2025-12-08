package frc.robot.components.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.chargers.misc.RobotMode;
import frc.chargers.misc.Tracer;
import frc.robot.components.vision.Structs.CameraConsts;
import frc.robot.components.vision.Structs.PoseEstimate;
import frc.robot.components.vision.Structs.RawCameraInputs;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import java.util.*;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.components.vision.VisionConsts.*;

public class Camera {
    private final CameraConsts consts;
    private final PhotonPoseEstimator poseEst;
    private final CameraIO io;
    private final RawCameraInputs inputs = new RawCameraInputs();

    private final List<Integer> fiducialIds = new ArrayList<>();
    private final List<Pose3d> poses = new ArrayList<>();

    public Camera(CameraConsts consts, Supplier<Pose2d> simPoseSupplier) {
        this.consts = consts;
        this.io = new CameraIO(consts, simPoseSupplier);
        this.poseEst = new PhotonPoseEstimator(
            consts.fieldLayout(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            consts.robotToCamera()
        );
        Logger.recordOutput(key("HasDebugLogs"), DEBUG_LOGS);
    }
    
    private String key(String path) {
        return "Cameras/" + consts.name() + "/" + path;
    }

    /**
     * Fetches the latest pose estimates from this camera,
     * using single-tag estimation if applicable.
     */
    public List<PoseEstimate> update(Rotation2d heading, double headingTimestampSecs) {
        poseEst.setMultiTagFallbackStrategy(
            DriverStation.isDisabled()
                ? PoseStrategy.LOWEST_AMBIGUITY
                : PoseStrategy.PNP_DISTANCE_TRIG_SOLVE
        );
        poseEst.addHeadingData(headingTimestampSecs, heading);
        return update();
    }

    /** Fetches the latest pose estimates from this camera. */
    public List<PoseEstimate> update() {
        Tracer.startTrace("Vision Update (" + consts.name() + ")");

        io.refreshData(inputs);
        Logger.processInputs(key(""), inputs);

        var poseEstimates = new ArrayList<PoseEstimate>();
        if (RobotMode.get() == RobotMode.REAL && !inputs.connected) {
            return poseEstimates;
        }

        int ambHighCount = 0;
        int errHighCount = 0;
        fiducialIds.clear();
        poses.clear();
        for (var result: inputs.results) {
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
                if (DEBUG_LOGS) fiducialIds.add(target.fiducialId);
            }
            if (ambiguityExceeded) {
                ambHighCount++;
                continue;
            }

            // updates the pose pose, and makes sure that the estimated pose
            // has a z coordinate near 0 and x and y coordinates within the field.
            var poseEstimate = poseEst.update(result);
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
            double linearStdDev = stdDevMultiplier * LINEAR_STD_DEV_BASELINE * consts.stdDevFactor();

            if (DEBUG_LOGS) poses.add(pose);
            var stdDevs = VecBuilder.fill(linearStdDev, linearStdDev, ANGULAR_STD_DEV);
            poseEstimates.add(new PoseEstimate(pose.toPose2d(), timestamp, stdDevs));
        }
        Tracer.endTrace();

        // logs relevant data
        if (DEBUG_LOGS) {
            int[] ids = new int[fiducialIds.size()];
            for (int i = 0; i < fiducialIds.size(); i++) {
                ids[i] = fiducialIds.get(i);
            }
            Logger.recordOutput(key("fiducialIds"), ids);
            Logger.recordOutput(key("numAmbiguityExceeded"), ambHighCount);
            Logger.recordOutput(key("numErrExceeded"), errHighCount);
            Logger.recordOutput(key("poses"), poses.toArray(new Pose3d[0]));
        }

        return poseEstimates;
    }
}