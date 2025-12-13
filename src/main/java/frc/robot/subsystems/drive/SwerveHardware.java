package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.chargers.misc.CurrAlliance;
import frc.robot.components.vision.Structs.CamPoseEstimate;
import frc.robot.subsystems.drive.SwerveData.OdometryFrame;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/** A class that wraps CTRE's {@link SwerveDrivetrain} with replay support. */
public class SwerveHardware {
    protected final SwerveDrivetrain<?, ?, ?> drivetrain;
    // prevents the poseEstBuffer from having items added & polled at the same time.
    private final Lock stateLock = new ReentrantLock();
    private final List<OdometryFrame> poseEstBuffer = new ArrayList<>();
    private SwerveDrivetrain.SwerveDriveState latest;

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
                var latestFrame = new OdometryFrame(
                    state.RawHeading, Utils.currentTimeToFPGATime(state.Timestamp),
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
        drivetrain.setOperatorPerspectiveForward(
            CurrAlliance.red() ? Rotation2d.k180deg : Rotation2d.kZero
        );
        inputs.heading = drivetrain.getRotation3d();
        try {
            stateLock.lock();
            inputs.poseEstFrames = new OdometryFrame[poseEstBuffer.size()];
            for (int i = 0; i < poseEstBuffer.size(); i++) {
                inputs.poseEstFrames[i] = poseEstBuffer.get(i);
            }
            poseEstBuffer.clear();
            inputs.currentStates = latest.ModuleStates;
            inputs.desiredStates = latest.ModuleTargets;
            inputs.notReplayedPose = latest.Pose;
            inputs.speeds = latest.Speeds;
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

    /** Adds a vision measurement to the non-replayed pose estimator. */
    public void addVisionMeasurement(CamPoseEstimate estimate) {
        drivetrain.addVisionMeasurement(
            estimate.pose(),
            Utils.fpgaToCurrentTime(estimate.timestampSecs()),
            estimate.deviations()
        );
    }

    /** Sets the pose estimation standard deviations for the encoder measurements. */
    public void setStateStdDevs(Matrix<N3, N1> stdDevs) {
        drivetrain.setStateStdDevs(stdDevs);
    }
}