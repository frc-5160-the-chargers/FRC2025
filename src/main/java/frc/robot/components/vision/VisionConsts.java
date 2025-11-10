package frc.robot.components.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;
import frc.chargers.data.RobotMode;
import frc.robot.constants.TunerConstants;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import java.util.List;
import java.util.Optional;

import static edu.wpi.first.units.Units.*;

public class VisionConsts {
    static final Transform3d FL_CAM_TRANSFORM = new Transform3d(
        Meters.of(TunerConstants.FrontLeft.LocationX - 0.026),
        Meters.of(TunerConstants.FrontLeft.LocationY + 0.11),
        Inches.of(7.375),
        new Rotation3d(
            Degrees.zero(),
            Degrees.of(-15),
            Degrees.of(48) // measured as: 46, previously working: 48
        )
    );
    static final Transform3d FR_CAM_TRANSFORM = new Transform3d(
        Meters.of(TunerConstants.FrontLeft.LocationX - 0.026),
        Meters.of(TunerConstants.FrontLeft.LocationY - 0.1),
        Inches.of(7.375),
        new Rotation3d(
            Degrees.zero(),
            Degrees.of(-15),
            Degrees.of(-48) // prob correct - another value -56
        )
    );

    static final Optional<VisionSystemSim> VISION_SYSTEM_SIM =
        RobotMode.isSim() ? Optional.of(new VisionSystemSim("main")) : Optional.empty();
    static final SimCameraProperties ARDUCAM_SIM_PROPERTIES = new SimCameraProperties();
    static final double MAX_AMBIGUITY = 0.2;
    static final Distance MAX_Z_ERROR = Meters.of(0.1);
    static final double Z_ERROR_SCALAR = 100.0;
    static final double SINGLE_TAG_SCALAR = 1.3;
    static final double LINEAR_STD_DEV_BASELINE = 0.4;
    static final double ANGULAR_STD_DEV = 10000000;

    static final AprilTagFieldLayout ALL_TAGS = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    static final AprilTagFieldLayout REEF_TAGS =
        new AprilTagFieldLayout(
            ALL_TAGS.getTags()
                .stream()
                .filter(it -> (it.ID >= 17 && it.ID <= 22) || (it.ID >= 6 && it.ID <= 11))
                .toList(),
            ALL_TAGS.getFieldLength(),
            ALL_TAGS.getFieldWidth()
        );
    static final AprilTagFieldLayout SOURCE_TAGS =
        new AprilTagFieldLayout(
            ALL_TAGS.getTags()
                .stream()
                .filter(it -> List.of(1, 2, 12, 13).contains(it.ID))
                .toList(),
            ALL_TAGS.getFieldLength(),
            ALL_TAGS.getFieldWidth()
        );

    static final Pose3d[] DUMMY_POSE_ARR = new Pose3d[0];
    static final String[] DUMMY_STRING_ARR = new String[0];

    static {
        ARDUCAM_SIM_PROPERTIES.setCalibration(1280, 800, Rotation2d.fromDegrees(70));
        ARDUCAM_SIM_PROPERTIES.setCalibError(0.25, 0.15);
        ARDUCAM_SIM_PROPERTIES.setAvgLatencyMs(35);
        ARDUCAM_SIM_PROPERTIES.setLatencyStdDevMs(5);
        ARDUCAM_SIM_PROPERTIES.setFPS(20);
        VISION_SYSTEM_SIM.ifPresent(it -> it.addAprilTags(ALL_TAGS));
    }
}
