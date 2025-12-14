package frc.robot.components.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.chargers.misc.RobotMode;
import frc.robot.components.vision.Structs.CameraConsts;
import frc.robot.components.vision.Structs.RawCameraInputs;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import java.util.Optional;
import java.util.function.Supplier;

/** A class that handles fetching vision frames(tracking position, etc) from a camera. */
public class CameraIO {
    private final PhotonCamera photonCam;
    private final Optional<VisionSystemSim> sim;
    private final Supplier<Pose2d> simPoseSupplier;

    public CameraIO(CameraConsts consts, Supplier<Pose2d> simPoseSupplier) {
        this.photonCam = new PhotonCamera(consts.name());
        this.simPoseSupplier = simPoseSupplier;
        if (RobotMode.isSim()) {
            this.sim = Optional.of(new VisionSystemSim(consts.name()));
            this.sim.get().addCamera(
                new PhotonCameraSim(photonCam, getProperties(consts), 0.12, 6.5),
                consts.robotToCamera()
            );
            this.sim.get().addAprilTags(consts.fieldLayout());
        } else {
            this.sim = Optional.empty();
        }
    }

    /** Refreshes RawCameraInputs with the latest frames. */
    public void refreshData(RawCameraInputs inputs) {
        if (RobotMode.get() == RobotMode.REPLAY) return;
        sim.ifPresent(it -> it.update(simPoseSupplier.get()));
        inputs.results = photonCam.getAllUnreadResults();
        inputs.connected = photonCam.isConnected();
    }

    private SimCameraProperties getProperties(CameraConsts consts) {
        var properties = new SimCameraProperties();
        if (consts.intrinsics().isPresent()) {
            var i = consts.intrinsics().get();
            properties.setCalibration(i.width(), i.height(), i.cameraMatrix(), i.distortionMatrix());
        } else { // otherwise, use default calibration
            properties.setCalibration(1280, 800, Rotation2d.fromDegrees(70));
        }
        properties.setCalibError(0.25, 0.15);
        properties.setAvgLatencyMs(35);
        properties.setLatencyStdDevMs(5);
        properties.setFPS(20);

        return properties;
    }
}
