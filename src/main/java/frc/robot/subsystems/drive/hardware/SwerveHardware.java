package frc.robot.subsystems.drive.hardware;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.chargers.misc.CurrAlliance;
import frc.chargers.misc.RobotMode;
import frc.robot.components.vision.Structs.CamPoseEstimate;
import frc.robot.subsystems.drive.hardware.SwerveData.OdometryFrame;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import static frc.robot.subsystems.drive.SwerveConsts.DRIVE_CONSTS_CHOICE;
import static frc.robot.subsystems.drive.SwerveConsts.MODULE_CONSTS_CHOICES;

/** A class that wraps CTRE's {@link SwerveDrivetrain} with replay support. */
public class SwerveHardware {
    protected final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain =
        new SwerveDrivetrain<>(
            TalonFX::new, TalonFX::new, CANcoder::new,
            DRIVE_CONSTS_CHOICE, MODULE_CONSTS_CHOICES
        );
    private final Lock stateLock = new ReentrantLock(); // prevents simultaneous modification of the poseEstBuffer
    private final List<OdometryFrame> poseEstBuffer = new ArrayList<>();
    private SwerveDrivetrain.SwerveDriveState latest = drivetrain.getStateCopy();

    public SwerveHardware() {
        if (RobotMode.get() == RobotMode.REPLAY) drivetrain.getOdometryThread().stop();
        // registerTelemetry() technically means "register a function that logs data",
        // but here we abuse it for data-gathering purposes.
        drivetrain.registerTelemetry(state -> {
            if (poseEstBuffer.size() > 30) return;
            try {
                stateLock.lock();
                latest = state;
                // phoenix 6 measures time differently, so we use currentTimeToFPGATime() to correct the timestamp.
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
        try {
            stateLock.lock();
            inputs.poseEstFrames = poseEstBuffer.toArray(new OdometryFrame[0]);
            poseEstBuffer.clear();
            inputs.currentStates = latest.ModuleStates;
            inputs.desiredStates = latest.ModuleTargets;
            inputs.notReplayedPose = latest.Pose;
            inputs.robotRelativeSpeeds = latest.Speeds;
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
            // phoenix 6 measures time differently, so we use fpgaToCurrentTime() to correct the timestamp.
            Utils.fpgaToCurrentTime(estimate.timestampSecs()),
            estimate.deviations()
        );
    }

    /** Sets the standard deviations of the encoder measurements; a.k.a how noisy they are. */
    public void setStateStdDevs(Matrix<N3, N1> stdDevs) {
        drivetrain.setStateStdDevs(stdDevs);
    }
}