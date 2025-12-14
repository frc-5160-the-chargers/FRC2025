package frc.robot.components.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.components.vision.Structs.CameraConsts;
import frc.robot.subsystems.drive.TunerConstants;

import java.util.Optional;

import static edu.wpi.first.units.Units.*;

public class VisionConsts {
    static boolean DEBUG_LOGS = true;

    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(
        AprilTagFields.k2025ReefscapeAndyMark
    );
    public static final AprilTagFieldLayout REEF_ONLY_LAYOUT = new AprilTagFieldLayout(
        FIELD_LAYOUT.getTags()
            .stream()
            .filter(it -> (it.ID >= 17 && it.ID <= 22) || (it.ID >= 6 && it.ID <= 11))
            .toList(),
        FIELD_LAYOUT.getFieldLength(),
        FIELD_LAYOUT.getFieldWidth()
    );

    public static final CameraConsts FL_CONSTS = new CameraConsts(
        "Chargers-FrontLeft",
        new Transform3d(
            Meters.of(TunerConstants.FrontLeft.LocationX - 0.026),
            Meters.of(TunerConstants.FrontLeft.LocationY + 0.11),
            Inches.of(7.375),
            new Rotation3d(
                Degrees.zero(),
                Degrees.of(-15),
                Degrees.of(48) // measured as: 46, previously working: 48
            )
        ),
        REEF_ONLY_LAYOUT, 1.0, Optional.empty()
    );
    public static final CameraConsts FR_CONSTS = new CameraConsts(
        "Chargers-FrontRight",
        new Transform3d(
            Meters.of(TunerConstants.FrontLeft.LocationX - 0.026),
            Meters.of(TunerConstants.FrontLeft.LocationY - 0.1),
            Inches.of(7.375),
            new Rotation3d(
                Degrees.zero(),
                Degrees.of(-15),
                Degrees.of(-48) // prob correct - another value -56
            )
        ),
        REEF_ONLY_LAYOUT, 1.0, Optional.empty()
    );

    static final double MAX_AMBIGUITY = 0.2;
    static final Distance MAX_Z_ERROR = Meters.of(0.1);
    static final double Z_ERROR_SCALAR = 100.0;
    static final double SINGLE_TAG_SCALAR = 1.3;
    static final double LINEAR_STD_DEV_BASELINE = 0.3;
    static final double ANGULAR_STD_DEV = 10000000;
}
