package frc.robot.subsystems.drive;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.chargers.misc.RobotMode;
import frc.chargers.misc.Tracer;
import frc.robot.components.vision.Structs.CamPoseEstimate;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.ChargerSubsystem;
import lombok.Getter;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.text.DecimalFormat;
import java.util.function.Supplier;

import static edu.wpi.first.math.MathUtil.angleModulus;
import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.SwerveConsts.*;

public class SwerveDrive extends ChargerSubsystem {
    private final SwerveDriveSimulation mapleSim =
        new SwerveDriveSimulation(MAPLESIM_CONFIG, Pose2d.kZero);
    private final SwerveDrivePoseEstimator replayPoseEst;
    private final SwerveSetpointGenerator setpointGen =
        new SwerveSetpointGenerator(PATH_PLANNER_CONFIG, STEER_MOTOR_TYPE.freeSpeedRadPerSec);
    private final RepulsorFieldPlanner repulsor = new RepulsorFieldPlanner();
    private final PIDController
        xPoseController = new PIDController(0, 0, 0.1),
        yPoseController = new PIDController(0, 0, 0.1),
        rotationController = new PIDController(0, 0, 0);

    // Persistent State
    private boolean replayPoseEstInitialized = false;
    private Pose2d goalToAlign = Pose2d.kZero;
    private SwerveSetpoint setpoint = NULL_SETPOINT;

    // Drive Requests
    private final SwerveRequest.ApplyFieldSpeeds fieldRelativeReq =
        new SwerveRequest.ApplyFieldSpeeds().withDriveRequestType(DriveRequestType.Velocity);

    // Hardware & Low-Level data
    private final SwerveHardware io = MapleSimSwerveHardware.from(
        mapleSim, TunerConstants.DrivetrainConstants,
        TunerConstants.FrontLeft, TunerConstants.FrontRight,
        TunerConstants.BackLeft, TunerConstants.BackRight
    );
    private final SwerveDataAutoLogged inputs = new SwerveDataAutoLogged();

    /** A pose estimate that will be replayed correctly. */
    @Getter
    @AutoLogOutput
    private Pose2d pose = Pose2d.kZero;

    public SwerveDrive() {
        var defaultPositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            defaultPositions[i] = new SwerveModulePosition();
        }
        replayPoseEst = new SwerveDrivePoseEstimator(
            new SwerveDriveKinematics(MODULE_TRANSLATIONS),
            Rotation2d.kZero, defaultPositions, Pose2d.kZero
        );
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    private SwerveModulePosition[] getModPositions() {
        return inputs.poseEstFrames[inputs.poseEstFrames.length - 1].positions();
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

    @Override
    public void loggedPeriodic() {
        io.refreshData(inputs);
        // If not in replay mode, logs every value.
        // If in replay mode, overrides every variable with values from the log file.
        Logger.processInputs("SwerveDrive", inputs);

        if (RobotMode.get() == RobotMode.REPLAY) {
            for (var frame: inputs.poseEstFrames) {
                if (!replayPoseEstInitialized) {
                    replayPoseEstInitialized = true;
                    replayPoseEst.resetPosition(
                        frame.heading(), frame.positions(), inputs.notReplayedPose
                    );
                }
                replayPoseEst.updateWithTime(
                    frame.timestampSecs(), frame.heading(), frame.positions()
                );
            }
            pose = replayPoseEst.getEstimatedPosition();
        } else {
            pose = inputs.notReplayedPose;
        }
        if (RobotMode.isSim()) Logger.recordOutput(key("TruePose"), truePose());
    }

    /** In sim, returns the true pose of the robot without odometry drift. */
    public Pose2d truePose() {
        return RobotMode.isSim() ? mapleSim.getSimulatedDriveTrainPose() : pose;
    }

    /**
     * Resets the pose of the robot. The pose should be from the
     * {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perspective.
     */
    public void resetPose(Pose2d pose) {
        io.resetNotReplayedPose(pose);
        if (RobotMode.get() != RobotMode.REPLAY) return;
        replayPoseEst.resetPosition(pose.getRotation(), getModPositions(), pose);
    }

    /** Returns a command that applies the given request repeatedly. */
    public Command driveCmd(Supplier<SwerveRequest> getRequest) {
        return this.run(() -> io.setControl(getRequest.get()))
            .withName("DriveCmd (" + getRequest.get().getClass().getSimpleName() + ")");
    }

    /** Returns a command that aligns the drivetrain to the pose while avoiding obstacles. */
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

    /** Adds a vision measurement to this drivetrain's pose estimator. */
    public void addVisionMeasurement(CamPoseEstimate estimate) {
        if (RobotMode.get() == RobotMode.REPLAY) {
            replayPoseEst.addVisionMeasurement(
                estimate.pose(), estimate.timestampSecs(), estimate.deviations()
            );
        }
        io.addVisionMeasurement(estimate);
    }

    /** Creates an AutoFactory, a utility class for following choreo trajectories. */
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
                    return; // don't log trajectory during matches, use replay mode to do so instead
                }
                Logger.recordOutput(
                    key("CurrentTraj/Samples"),
                    trajectory.samples().toArray(new SwerveSample[0])
                );
            }
        );
    }

    private static class WheelRadiusCharacterizationState {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        Rotation2d lastAngle = Rotation2d.kZero;
        double gyroDelta = 0.0;
    }

    /** Measures the robot's wheel radius by spinning in a circle. */
    public Command wheelRadiusCharacterization() {
        var limiter = new SlewRateLimiter(0.05);
        var state = new WheelRadiusCharacterizationState();
        var req = new SwerveRequest.RobotCentric();
        var movementCmd = Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(() -> limiter.reset(0.0)),
            // Turn in place, accelerating up to full speed
            driveCmd(() -> req.withRotationalRate(limiter.calculate(0.25)))
        );
        var measurementCmd = Commands.sequence(
            Commands.waitSeconds(1.0),
            Commands.runOnce(() -> {
                state.positions = getModPositions();
                state.gyroDelta = 0.0;
                state.lastAngle = inputs.heading.toRotation2d();
            }),
            Commands.run(() -> {
                var rotation = inputs.heading.toRotation2d();
                state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRotations());
                state.lastAngle = rotation;
            })
        ).finallyDo(() -> {
            var currPositions = getModPositions();
            double wheelDeltaRots = 0.0;
            for (int i = 0; i < 4; i++) {
                wheelDeltaRots += Math.abs(
                    currPositions[i].distanceMeters - state.positions[i].distanceMeters
                ) / 4.0;
            }
            double wheelRadius = (state.gyroDelta * DRIVE_BASE_RADIUS.in(Meter)) / wheelDeltaRots;
            var formatter = new DecimalFormat("#0.000000000000000000000000000");
            System.out.println("********** Wheel Radius Characterization Results **********");
            System.out.println("\tWheel Delta: " + formatter.format(wheelDeltaRots) + " rotations");
            System.out.println("\tGyro Delta: " + formatter.format(state.gyroDelta) + " rotations");
            System.out.println("\tWheel Radius: " + formatter.format(wheelRadius) + " meters");
        });
        return Commands.parallel(movementCmd, measurementCmd);
    }
}
