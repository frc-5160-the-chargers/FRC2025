package choreo;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

/**
 * File containing values filled into the document settings for your choreo project.
 * This allows for modifying constants in choreo while keeping your robot code up-to-date.
 * DO NOT MODIFY this file yourself, as it is auto-generated.
 */
public final class ChoreoConsts {
    public static final double gearing = 6.2;
    public static final double frictionCoefficient = 1.5;
    public static final Distance wheelRadius = Meters.of(0.0508);
    public static final MomentOfInertia moi = KilogramSquareMeters.of(5.8);
    public static final Mass mass = Kilograms.of(99.0);
    public static final Torque driveMotorMaxTorque = NewtonMeters.of(0.4);
    public static final Distance wheelBaseWithBumpers = Meters.of(1.0032999999999999);
    public static final Distance trackWidthWithBumpers = Meters.of(0.8636);
    public static final Translation2d[] moduleTranslations = {
        new Translation2d(0.41275, 0.3429),
        new Translation2d(0.41275, -0.3429),
        new Translation2d(-0.41275, 0.3429),
        new Translation2d(-0.41275, -0.3429),
    };

    public static final AngularVelocity maxAngularVel = RadiansPerSecond.of(9.593976477189083);
    public static final AngularAcceleration maxAngularAccel = RadiansPerSecondPerSecond.of(18.066466002427948);
    public static final LinearVelocity maxLinearVel = MetersPerSecond.of(5.1481582839471445);
    public static final LinearAcceleration maxLinearAccel = MetersPerSecondPerSecond.of(1.9724807126381936);

    private ChoreoConsts() {}
}
