package frc.robot.subsystems.drivev3;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.chargers.misc.RobotMode;
import frc.chargers.misc.Tracer;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.ChargerSubsystem;
import frc.robot.subsystems.drive.RepulsorFieldPlanner;
import lombok.Getter;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static edu.wpi.first.math.MathUtil.angleModulus;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.subsystems.drivev3.SwerveConsts.*;

public class SwerveDrive extends ChargerSubsystem {
    // The physics sim.
    private final SwerveDriveSimulation mapleSim =
        new SwerveDriveSimulation(MAPLESIM_CONFIG, Pose2d.kZero);
    // To support vision tuning, we re-calculate the pose estimate manually in replay mode.
    private final SwerveDrivePoseEstimator replayPoseEst;
    // Smooths out speed requests to the drivetrain, allowing for
    private final SwerveSetpointGenerator setpointGen =
        new SwerveSetpointGenerator(PATH_PLANNER_CONFIG, STEER_MOTOR_TYPE.freeSpeedRadPerSec);
    private final RepulsorFieldPlanner repulsor = new RepulsorFieldPlanner();
    private final PIDController
        xPoseController = new PIDController(0, 0, 0.1),
        yPoseController = new PIDController(0, 0, 0.1),
        rotationController = new PIDController(0, 0, 0);

    // Persistent State
    private final SwerveModulePosition[] positions = new SwerveModulePosition[4];
    private Pose2d goalToAlign = Pose2d.kZero;
    private SwerveSetpoint setpoint = NULL_SETPOINT;
    private boolean poseEstInit = false;

    // Drive Requests
    private final SwerveRequest.ApplyFieldSpeeds fieldRelativeReq =
        new SwerveRequest.ApplyFieldSpeeds().withDriveRequestType(DriveRequestType.Velocity);

    // Hardware & Low-Level data
    private final SwerveHardware io = SwerveHardware.from(
        TunerConstants.DrivetrainConstants,
        TunerConstants.FrontLeft, TunerConstants.FrontRight,
        TunerConstants.BackLeft, TunerConstants.BackRight
    );
    private final SwerveDataAutoLogged inputs = new SwerveDataAutoLogged();

    /** A pose estimate that will be replayed correctly. */
    @Getter
    @AutoLogOutput
    private Pose2d pose = Pose2d.kZero;

    public SwerveDrive() {
        for (int i = 0; i < 4; i++) {
            positions[i] = new SwerveModulePosition();
        }
        replayPoseEst = new SwerveDrivePoseEstimator(
            new SwerveDriveKinematics(MODULE_TRANSLATIONS),
            Rotation2d.kZero, positions, Pose2d.kZero
        );
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    private void driveWithSample(SwerveSample sample, boolean useSetpointGen) {
        xPoseController.setP(TRANSLATION_KP.get());
        yPoseController.setP(TRANSLATION_KP.get());
        rotationController.setPID(ROTATION_KP.get(), 0, ROTATION_KD.get());
        var speeds = sample.getChassisSpeeds();
        speeds.vxMetersPerSecond += xPoseController.calculate(pose.getX(), sample.x);
        speeds.vyMetersPerSecond += yPoseController.calculate(pose.getY(), sample.y);
        speeds.omegaRadiansPerSecond += rotationController.calculate(
            angleModulus(inputs.heading.getZ()),
            angleModulus(sample.heading)
        );
        fieldRelativeReq.Speeds = speeds;
        if (useSetpointGen) {
            var robotRelativeSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(speeds, inputs.heading.toRotation2d());
            setpoint = setpointGen.generateSetpoint(setpoint, robotRelativeSpeeds, 0.02);
            fieldRelativeReq.WheelForceFeedforwardsX =
                setpoint.feedforwards().robotRelativeForcesXNewtons();
            fieldRelativeReq.WheelForceFeedforwardsY =
                setpoint.feedforwards().robotRelativeForcesYNewtons();
        } else {
            fieldRelativeReq.WheelForceFeedforwardsX = sample.moduleForcesX();
            fieldRelativeReq.WheelForceFeedforwardsY = sample.moduleForcesY();
        }
        io.setControl(fieldRelativeReq);
    }

    private void updatePositions(SwerveData.PoseEstimationFrame frame) {
        positions[0] = frame.tl();
        positions[1] = frame.tr();
        positions[2] = frame.bl();
        positions[3] = frame.br();
    }

    @Override
    public void loggedPeriodic() {
        io.refreshData(inputs);
        Logger.processInputs("SwerveDrive", inputs);

        for (var frame: inputs.poseEstFrames) {
            updatePositions(frame);
            replayPoseEst.updateWithTime(
                frame.timestampSecs(), frame.heading(), positions
            );
        }
        Logger.recordOutput(key("ManualPose"), replayPoseEst.getEstimatedPosition());
        if (RobotMode.get() == RobotMode.REPLAY) {
            pose = replayPoseEst.getEstimatedPosition();
        } else {
            pose = inputs.notReplayedPose;
        }
    }

    /**
     * Resets the pose of the robot. The pose should be from the
     * {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perspective.
     */
    public void resetPose(Pose2d pose) {
        replayPoseEst.resetPose(pose);
        io.resetNotReplayedPose(pose);
    }

    /** Returns a command that applies the given request repeatedly. */
    public Command driveCmd(Supplier<SwerveRequest> getRequest) {
        return this.run(() -> io.setControl(getRequest.get()))
            .withName("DriveCmd (" + getRequest.get().getClass().getSimpleName() + ")");
    }

    /**
     * Returns a command that pathfinds the drivetrain to the correct pose.
     */
    public Command pathfindCmd(Supplier<Pose2d> targetPoseSupplier) {
        double maxSpeedMps = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        return Tracer.trace(
            "Repulsor Pathfind Cmd",
            this.run(() -> {
                goalToAlign = targetPoseSupplier.get();
                repulsor.setGoal(goalToAlign);
                var sample = repulsor.sampleField(pose.getTranslation(), maxSpeedMps, 1.5);
                driveWithSample(sample, true);
            }).beforeStarting(() -> setpoint = NULL_SETPOINT)
        );
    }

    public void addVisionMeasurement(Pose2d visionPose, double visionTimestamp, Matrix<N3, N1> stdDevs) {
        if (RobotMode.get() == RobotMode.REPLAY) {
            replayPoseEst.addVisionMeasurement(
                visionPose, visionTimestamp + inputs.timeOffsetSecs, stdDevs);
        } else {
            io.addVisionMeasurement(visionPose, Utils.fpgaToCurrentTime(visionTimestamp), stdDevs);
        }
    }

    public AutoFactory createAutoFactory() {
        return new AutoFactory(
            () -> pose,
            this::resetPose,
            // a function that runs trajectory following
            (SwerveSample trajSample) -> driveWithSample(trajSample, false),
            true,
            this,
            (trajectory, isStart) -> {
                Logger.recordOutput(key("CurrentTraj/Name"), trajectory.name());
                if (RobotMode.get() != RobotMode.REPLAY && DriverStation.isFMSAttached()) {
                    return;
                }
                Logger.recordOutput(
                    key("CurrentTraj/Samples"),
                    trajectory.samples().toArray(new SwerveSample[0])
                );
            }
        );
    }
}
