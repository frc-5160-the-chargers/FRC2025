package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.drive.SwerveData.PoseEstimationFrame;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/** A class that wraps CTRE's {@link SwerveDrivetrain} with replay support. */
public class SwerveHardware {
    // The underlying CTRE swerve drivetrain that we are wrapping.
    protected final SwerveDrivetrain<?, ?, ?> drivetrain;
    private final Lock stateLock = new ReentrantLock();
    private final List<PoseEstimationFrame> poseEstBuffer = new ArrayList<>();
    private SwerveDriveState latest;

    public SwerveHardware(SwerveDrivetrain<?, ?, ?> drivetrain) {
        this.drivetrain = drivetrain;
        latest = drivetrain.getStateCopy();
        // registerTelemetry() technically means "register a function that logs the data",
        // but here we abuse it for data-gathering purposes.
        drivetrain.registerTelemetry(state -> {
            if (poseEstBuffer.size() > 20) return;
            try {
                stateLock.lock();
                latest = state;
                var latestFrame = new PoseEstimationFrame(
                    state.RawHeading, Utils.fpgaToCurrentTime(state.Timestamp),
                    state.ModulePositions[0], state.ModulePositions[1],
                    state.ModulePositions[2], state.ModulePositions[3]
                );
                poseEstBuffer.add(latestFrame);
            } finally {
                stateLock.unlock();
            }
        });
    }

    /** Updates a {@link SwerveDataAutoLogged} instance with the latest data. */
    public void refreshData(SwerveDataAutoLogged inputs) {
        inputs.timeOffsetSecs = Utils.fpgaToCurrentTime(0);
        inputs.heading = drivetrain.getRotation3d();
        try {
            stateLock.lock();
            inputs.poseEstFrames = new PoseEstimationFrame[poseEstBuffer.size()];
            for (int i = 0; i < poseEstBuffer.size(); i++) {
                inputs.poseEstFrames[i] = poseEstBuffer.get(i);
            }
            poseEstBuffer.clear();
            inputs.currentStates = latest.ModuleStates;
            inputs.desiredStates = latest.ModuleTargets;
            inputs.notReplayedPose = latest.Pose;
        } finally {
            stateLock.unlock();
        }
    }

    /** Applies the specified control request to this swerve drivetrain. */
    public void setControl(SwerveRequest request) {
        drivetrain.setControl(request);
    }

    /** Resets the non-replayed pose. */
    public void resetNotReplayedPose(Pose2d pose) {
        drivetrain.resetPose(pose);
    }

    /** Adds a vision measurement to the drivetrain's native pose estimator. */
    public void addVisionMeasurement(
        Pose2d visionPose,
        double timestampSecs,
        Matrix<N3, N1> visionStandardDeviations
    ) {
        drivetrain.addVisionMeasurement(
            visionPose, timestampSecs,
            visionStandardDeviations
        );
    }

    /** Sets the pose estimation standard deviations for the encoder measurements. */
    public void setStateStdDevs(Matrix<N3, N1> stdDevs) {
        drivetrain.setStateStdDevs(stdDevs);
    }

    /**
     * If the operator is in the Blue Alliance Station, this should be 0 degrees.
     * If the operator is in the Red Alliance Station, this should be 180 degrees.
     * @see SwerveDrivetrain#setOperatorPerspectiveForward
     */
    public void setOperatorPerspectiveForward(Rotation2d fieldDirection) {
        drivetrain.setOperatorPerspectiveForward(fieldDirection);
    }
}