package frc.robot.components.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/** Data structures used for vision processing. */
public class Structs {
    /** Camera-Specific Constants. */
    public record CameraConsts(
        String name,
        Transform3d robotToCamera,
        AprilTagFieldLayout fieldLayout,
        double stdDevFactor,
        Optional<CameraIntrinsics> intrinsics
    ) {}

    /** A pose estimate originating from the vision cameras. */
    public record CamPoseEstimate(Pose2d pose, double timestampSecs, Vector<N3> deviations) {}

    /** Represents raw camera inputs from a photon camera. */
    public static class RawCameraInputs implements LoggableInputs {
        public boolean connected = true;
        public List<PhotonPipelineResult> results = new ArrayList<>();

        /** Pushes data to a log file. */
        @Override
        public void toLog(LogTable logTable) {
            logTable.put("connected", connected);
            logTable.put("numResults", results.size());
            for (int i = 0; i < results.size(); i++) {
                logTable.put("results/" + i, results.get(i));
            }
        }

        /** Overrides the results with data from another file, effectively "replaying" a previous log. */
        @Override
        public void fromLog(LogTable logTable) {
            results.clear();
            connected = logTable.get("connected", false);
            int numResults = logTable.get("numResults", 0);
            for (int i = 0; i < numResults; i++) {
                results.add(logTable.get("results/" + i, new PhotonPipelineResult()));
            }
        }
    }

    /** Calibration data for a camera. */
    public record CameraIntrinsics(
        int width,
        int height,
        double fx,
        double fy,
        double cx,
        double cy,
        double[] distortion
    ) {
        public CameraIntrinsics(double fx, double fy, double cx, double cy, double[] distortion) {
            this(1280, 800, fx, fy, cx, cy, distortion);
        }

        public Matrix<N8, N1> distortionMatrix() {
            return MatBuilder.fill(Nat.N8(), Nat.N1(), distortion);
        }

        public Matrix<N3, N3> cameraMatrix() {
            return MatBuilder.fill(Nat.N3(), Nat.N3(), fx, 0, cx, 0, fy, cy, 0, 0, 1);
        }

        public double horizontalFOV() {
            return 2.0 * Math.atan2(width, 2.0 * fx);
        }

        public double verticalFOV() {
            return 2.0 * Math.atan2(height, 2.0 * fy);
        }

        public double diagonalFOV() {
            return 2.0 * Math.atan2(Math.hypot(width, height) / 2.0, fx);
        }
    }

    /** This is a utility class. */
    private Structs() {}
}
