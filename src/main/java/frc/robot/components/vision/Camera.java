package frc.robot.components.vision;

import edu.wpi.first.math.geometry.Transform3d;
import frc.chargers.data.RobotMode;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;

import static frc.robot.components.vision.VisionConsts.ARDUCAM_SIM_PROPERTIES;
import static frc.robot.components.vision.VisionConsts.VISION_SYSTEM_SIM;

public class Camera {
    private final PhotonCamera photonCam;

    /** The distance between the robot's center and the lens center. */
    public final Transform3d robotToCamera;
    /** The higher this number is, the less the camera's data is trusted. */
    public final double stdDevFactor;
    /** The current camera data. */
    public final RawCameraData inputs = new RawCameraData();

    public Camera(String name, double stdDevFactor, Transform3d robotToCamera) {
        this.photonCam = new PhotonCamera(name);
        this.stdDevFactor = stdDevFactor;
        this.robotToCamera = robotToCamera;

        VISION_SYSTEM_SIM.ifPresent(
            it -> it.addCamera(
                new PhotonCameraSim(photonCam, ARDUCAM_SIM_PROPERTIES),
                robotToCamera
            )
        );
    }

    /** Fetches a relative log path for this camera. */
    public String logKey(String path) {
        return "Cameras/" + photonCam.getName() + "/" + path;
    }

    /** Must be called periodically(updates the necessary data) */
    public void refreshData() {
        if (RobotMode.get() != RobotMode.REPLAY) {
            inputs.results = photonCam.getAllUnreadResults();
            inputs.connected = photonCam.isConnected();
        }
    }
}
