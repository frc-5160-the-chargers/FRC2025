package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

/**
 * Generated file containing variables defined in Choreo.
 * DO NOT MODIFY THIS FILE YOURSELF; instead, change these values in the Choreo GUI.
 */
public final class ChoreoVars {
    public static final MomentOfInertia botMOI = KilogramSquareMeters.of(5.8);
    public static final Mass botMass = Kilograms.of(100);
    public static final double cof = 1.5;
    public static final double gearing = 6.2;
    public static final Distance moduleX = Meters.of(0.348);
    public static final Distance moduleY = Meters.of(0.279);
    public static final Distance wheelRadius = Meters.of(0.051);

    private ChoreoVars() {}
}