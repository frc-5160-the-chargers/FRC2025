package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

/**
 * Data received from a swerve drivetrain every 0.02 seconds.
 * Use {@link SwerveDataAutoLogged} in place of this class.
 */
@AutoLog
public class SwerveData {
    public OdometryFrame[] poseEstFrames = {};
    public SwerveModuleState[] currentStates = new SwerveModuleState[4];
    public SwerveModuleState[] desiredStates = new SwerveModuleState[4];
    public ChassisSpeeds speeds = new ChassisSpeeds();
    public Rotation3d heading = Rotation3d.kZero;
    public Pose2d notReplayedPose = Pose2d.kZero;

    /** Data used for estimating pose in replay mode. */
    public record OdometryFrame(
        Rotation2d heading,
        double timestampSecs,
        SwerveModulePosition tl,
        SwerveModulePosition tr,
        SwerveModulePosition bl,
        SwerveModulePosition br
    ) {
        public SwerveModulePosition[] positions() {
            return new SwerveModulePosition[] {tl, tr, bl, br};
        }
    }
}
