package frc.chargers.utils;

public class EpsilonEquals {
    private static final double EPSILON = 1E-9;
    private EpsilonEquals() {}

    /**
     * Checks if 2 doubles are equal; correcting for floating point error.
     * Note: cannot be used as extension due to the double type being primitive.
     * Usage: <code>equivalent(2.0, 3.0 - 1.0)</code>
     */
    public static boolean equivalent(double a, double b) {
        return Math.abs(a - b) <= EPSILON;
    }
}
