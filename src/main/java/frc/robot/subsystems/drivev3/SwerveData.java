package frc.robot.subsystems.drivev3;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

/**
 * Data received from a swerve drivetrain every 0.02 seconds.
 */
@AutoLog
public class SwerveData {
    public PoseEstimationFrame[] poseEstFrames = {};
    public SwerveModuleState[] currentStates = new SwerveModuleState[4];
    public SwerveModuleState[] desiredStates = new SwerveModuleState[4];
    public Rotation3d heading = Rotation3d.kZero;
    public Pose2d notReplayedPose = Pose2d.kZero;
    // Phoenix 6 uses a custom timestamp, so we have to log and replay it here.
    public double timeOffsetSecs = 0.0;

    /** Data used for estimating pose in replay mode. */
    public record PoseEstimationFrame(
        Rotation2d heading,
        double timestampSecs,
        SwerveModulePosition tl,
        SwerveModulePosition tr,
        SwerveModulePosition bl,
        SwerveModulePosition br
    ) {}
}
