package choreo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

/**
 * File containing variables created in choreo,
 * allowing for value changes in choreo to be reflected in robot code.
 * DO NOT MODIFY this file yourself, as it is auto-generated.
 */
public final class ChoreoVars {
    public static final LinearAcceleration hi = MetersPerSecondPerSecond.of(5.0);
    public static final MomentOfInertia helloThere = KilogramSquareMeters.of(5.0);
    public static final double numTest = 500.0;
    public static final Distance offset = Meters.of(0.15239999999999998);
    public static final double justANum = 3.0;
    public static final AngularVelocity sdfsdf = RadiansPerSecond.of(5.0);

    public static final class Poses {
        public static final Pose2d reef1 = new Pose2d(2.9910531999999996, 4.2204005, Rotation2d.kZero);
        public static final Pose2d southSource = new Pose2d(1.5917405154734423, 0.8619509473501505, Rotation2d.fromRadians(-2.198922871295136));
        public static final Pose2d reef0 = new Pose2d(2.9910531999999996, 3.891775300000001, Rotation2d.kZero);
        public static final Pose2d reef3 = new Pose2d(3.908621177003519, 5.420724655568287, Rotation2d.fromRadians(-1.047197551196598));
        public static final Pose2d reef2 = new Pose2d(3.6240234054797775, 5.256412055568287, Rotation2d.fromRadians(-1.047197551196598));
        public static final Pose2d reef9 = new Pose2d(5.07007562299648, 2.631126144431714, Rotation2d.fromRadians(2.0943951023931957));
        public static final Pose2d reef8 = new Pose2d(5.354673394520223, 2.795438744431714, Rotation2d.fromRadians(2.0943951023931957));
        public static final Pose2d reef11 = new Pose2d(3.571780422996481, 2.825601244431714, Rotation2d.fromRadians(1.047197551196598));
        public static final Pose2d reef10 = new Pose2d(3.856378194520223, 2.661288644431714, Rotation2d.fromRadians(1.047197551196598));
        public static final Pose2d reef5 = new Pose2d(5.40691637700352, 5.226249555568287, Rotation2d.fromRadians(-2.0943951023931957));
        public static final Pose2d reef4 = new Pose2d(5.122318605479777, 5.390562155568286, Rotation2d.fromRadians(-2.0943951023931957));
        public static final Pose2d reef7 = new Pose2d(5.9876436, 3.8314503, Rotation2d.fromRadians(3.141592653589793));
        public static final Pose2d reef6 = new Pose2d(5.9876436, 4.1600755, Rotation2d.fromRadians(3.141592653589793));
        public static final Pose2d northSource = new Pose2d(1.5917405154734423, 7.189849052649849, Rotation2d.fromRadians(2.198922871295136));

        private Poses() {}
    }

    private ChoreoVars() {}
}
