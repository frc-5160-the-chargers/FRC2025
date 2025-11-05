package frc.robot.subsystems.drive;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.chargers.data.CurrAlliance;
import frc.chargers.data.RobotMode;
import frc.robot.components.gyro.Gyro;
import frc.robot.components.gyro.GyroDataAutoLogged;
import frc.robot.components.gyro.GyroPigeon2;
import frc.robot.components.gyro.GyroSim;
import frc.robot.components.vision.AprilTagVision.PoseEstimate;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.ChargerSubsystem;
import frc.robot.subsystems.drive.module.SwerveModule;
import frc.chargers.misc.RepulsorFieldPlanner;
import frc.chargers.misc.Tracer;
import frc.chargers.data.InputStream;
import lombok.Getter;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.Arrays;
import java.util.function.Supplier;

import static choreo.util.ChoreoAllianceFlipUtil.flip;
import static edu.wpi.first.math.MathUtil.angleModulus;
import static edu.wpi.first.math.util.Units.metersToInches;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj.Alert.AlertType.kError;
import static frc.chargers.commands.TriggerUtil.bind;
import static frc.robot.subsystems.drive.SwerveConsts.*;

/**
 * A drivetrain with 4 drive motors and 4 steer motors.
 * Each steer motor can control the exact position of each drive motor,
 * allowing for omnidirectional movement and driving while turning. <br/>
 *
 * Note: for pose estimation to work, you must call drivetrain.updateOdometry()
 * in robotPeriodic or through an addPeriodic() call for higher frequency.
 */
public class SwerveDrive extends ChargerSubsystem {
    private final SwerveModulePosition[] measuredModulePositions = new SwerveModulePosition[4];

    // kinematics and pose estimator
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(MODULE_TRANSLATIONS);
    private final SwerveDrivePoseEstimator poseEstimator;

    // simulation and path planning
    private final SwerveDriveSimulation mapleSim =
        new SwerveDriveSimulation(MAPLESIM_CONFIG, Pose2d.kZero);
    private final RepulsorFieldPlanner repulsor = new RepulsorFieldPlanner();
    private final SwerveSetpointGenerator setpointGen = new SwerveSetpointGenerator(
        PATH_PLANNER_CONFIG, STEER_MOTOR_TYPE.freeSpeedRadPerSec
    );
    private final PIDController
        xPoseController = new PIDController(TRANSLATION_KP, 0, 0),
        yPoseController = new PIDController(TRANSLATION_KP, 0, 0),
        rotationController = new PIDController(ROTATION_KP, 0, ROTATION_KD);

    private final SwerveModule[] swerveModules = {
        new SwerveModule("FL", mapleSim.getModules()[0], TunerConstants.FrontLeft),
        new SwerveModule("FR", mapleSim.getModules()[1], TunerConstants.FrontRight),
        new SwerveModule("BL", mapleSim.getModules()[2], TunerConstants.BackLeft),
        new SwerveModule("BR", mapleSim.getModules()[3], TunerConstants.BackRight)
    };
    private final Gyro gyro = switch (RobotMode.get()) {
        case REAL -> new GyroPigeon2();
        case SIM -> new GyroSim(mapleSim.getGyroSimulation());
        case REPLAY -> new Gyro();
    };
    @Getter private final GyroDataAutoLogged gyroInputs = new GyroDataAutoLogged();

    // Persistent State
    private Pose2d goalToAlign = Pose2d.kZero;
    private Rotation2d rawGyroRotation = new Rotation2d();
    private SwerveSetpoint setpoint = NULL_SETPOINT;

    /** A SysId routine for characterization translational velocity/accel. */
    public final SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.per(Second).of(0.5),
            Volts.of(3),
            Seconds.of(10),
            state -> Logger.recordOutput("swerve/sysIdState", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            voltage -> {
                for (var module: swerveModules) {
                    module.driveStraight(voltage.in(Volts));
                }
            },
            log -> {},
            this
        )
    );

    public SwerveDrive() {
        // module states & positions
        Arrays.fill(measuredModulePositions, new SwerveModulePosition());
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, Rotation2d.kZero, measuredModulePositions, Pose2d.kZero);

        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
        this.xPoseController.setTolerance(TRANSLATION_TOLERANCE);
        this.yPoseController.setTolerance(TRANSLATION_TOLERANCE);
        this.rotationController.setTolerance(ROTATION_TOLERANCE);

        if (RobotBase.isSimulation()) {
            SimulatedArena.getInstance().addDriveTrainSimulation(mapleSim);
        }
        OdoThread.getInstance().start();

        bind(
            new Alert("Gyro has error", kError),
            () -> !gyroInputs.connected
        );
    }

    /** Obtains desired module states from a ChassisSpeeds target. */
    private void driveCallback(ChassisSpeeds speeds, boolean useSetpointGen) {
        Logger.recordOutput(key("desiredSpeeds"), speeds);
        Logger.recordOutput(key("setpointGenUsed"), useSetpointGen);
        SwerveModuleState[] desiredStates;
        if (useSetpointGen) {
            setpoint = setpointGen.generateSetpoint(setpoint, speeds, 0.02);
            desiredStates = setpoint.moduleStates();
        } else {
            speeds = ChassisSpeeds.discretize(speeds, 0.02);
            desiredStates = kinematics.toSwerveModuleStates(speeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, TunerConstants.kSpeedAt12Volts);
            setpoint = new SwerveSetpoint(speeds, desiredStates, NULL_SETPOINT.feedforwards());
        }
        Logger.recordOutput(key("desiredStates"), setpoint.moduleStates());
        for (int i = 0; i < 4; i++) {
            var currentAngle = swerveModules[i].getAngle();
            desiredStates[i].optimize(currentAngle);
            desiredStates[i].cosineScale(currentAngle);
            swerveModules[i].runSetpoint(desiredStates[i]);
        }
    }

    /** Obtains desired module states from a choreo trajectory sample. */
    private ChassisSpeeds toDesiredSpeeds(SwerveSample trajSample, double linearVelMultiplier) {
        var vx = trajSample.vx + xPoseController.calculate(poseEstimate().getX(), trajSample.x);
        var vy = trajSample.vy + yPoseController.calculate(poseEstimate().getY(), trajSample.y);
        var rotationV = trajSample.omega + rotationController.calculate(
            angleModulus(bestPose().getRotation().getRadians()),
            angleModulus(trajSample.heading)
        );
        vx *= linearVelMultiplier;
        vy *= linearVelMultiplier;
        return ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rotationV, bestPose().getRotation());
    }

    /** Creates a choreo AutoFactory. You should cache this in your Robot or AutoCommands class. */
    public AutoFactory createAutoFactory() {
        return new AutoFactory(
            this::poseEstimate,
            this::resetPose,
            // a function that runs trajectory following
            (SwerveSample trajSample) -> driveCallback(toDesiredSpeeds(trajSample, 1), false),
            true,
            this,
            (trajectory, isStart) -> {
                Logger.recordOutput(
                    "CurrentTraj/samples", trajectory.samples().toArray(new SwerveSample[0])
                );
                Logger.recordOutput("CurrentTraj/name", trajectory.name());
            }
        );
    }

    public Command runDriveMotors() {
        return this.run(() -> {
            for (var mod: swerveModules) {
                mod.driveStraight(0.1);
            }
        }).withName("RunDriveMotors");
    }

    public Command runTurnMotors() {
        return this.run(() -> {
            double steerVolts = DEMO_STEER_VOLTS.get();
            for (var mod: swerveModules) {
                mod.spinAzimuth(steerVolts);
            }
        }).withName("RunTurnMotors");
    }

    public Command setSteerAngles(Rotation2d... angles) {
        if (angles.length == 1) {
            return this.run(() -> {
                for (var mod: swerveModules) {
                    mod.runSetpoint(new SwerveModuleState(0, angles[0]));
                }
            }).withName("SetSteerAngles");
        } else if (angles.length == 4) {
            return this.run(() -> {
                for (int i = 0; i < 4; i++) {
                    swerveModules[i].runSetpoint(new SwerveModuleState(0, angles[i]));
                }
            });
        } else {
            return Commands.print("Invalid amount of steer angles - command ignored.");
        }
    }

    /** Gets the drivetrain's calculated pose pose. */
    @AutoLogOutput
    public Pose2d poseEstimate() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * In simulation, gets the actual pose of the drivetrain
     * (ignoring simulated motor drift, collisions, and inaccuracy) - calculated by maple sim
     * On the real robot, returns the same thing as poseEstimate(). <br />
     * Note that you shouldn't use this method most of the time, as we want to simulate
     * encoder drift in sim.
     */
    @AutoLogOutput(key = "SwerveDrive/actualPose(if run in sim)")
    public Pose2d bestPose() {
        return RobotBase.isSimulation() ? mapleSim.getSimulatedDriveTrainPose() : poseEstimate();
    }

    public void resetPose(Pose2d pose) {
        if (RobotBase.isSimulation()) {
            // don't reset rotation in sim, as maple sim's value has already changed
            poseEstimator.resetTranslation(pose.getTranslation());
            mapleSim.setSimulationWorldPose(pose);
        } else {
            poseEstimator.resetPose(pose);
        }
    }

    public Pose2d getDemoPose() {
        return new Pose2d(
            DEMO_POSE_X.get(), DEMO_POSE_Y.get(),
            Rotation2d.fromDegrees(DEMO_POSE_HEADING_DEG.get())
        );
    }

    /**
     * Creates a Command that, when scheduled,
     * drives the robot forever at the requested forward, strafe, and rotation powers.
     * Does not move the robot at an exact velocity; so best used in teleop
     * where exact velocities are not important.
     * @see InputStream
     */
    public Command driveCmd(
        InputStream forwardOutput,
        InputStream strafeOutput,
        InputStream rotationOutput,
        boolean fieldRelative
    ) {
        var maxSpeedMps = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        return this.run(() -> {
            var rotation = Rotation2d.kZero;
            if (fieldRelative) {
                rotation = poseEstimate().getRotation();
                if (CurrAlliance.red()) rotation = flip(rotation);
            }
            var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                forwardOutput.get() * maxSpeedMps,
                strafeOutput.get() * maxSpeedMps,
                rotationOutput.get() * maxSpeedMps,
                rotation
            );
            driveCallback(speeds, true);
        }).withName("SwerveDriveCmd(Open Loop)");
    }

    /**
     * Returns a command that pathfinds the drivetrain to the correct pose.
     * @return a Command
     */
    public Command pathfindCmd(Supplier<Pose2d> targetPoseSupplier) {
        double maxVelocityMps = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        return this.run(() -> {
            Tracer.startTrace("repulsor pathfinding");
            goalToAlign = targetPoseSupplier.get();
            repulsor.setGoal(goalToAlign);
            var sample = repulsor.sampleField(poseEstimate().getTranslation(), maxVelocityMps * .8, 1.5);
            var desiredSpeeds = toDesiredSpeeds(sample, 1.8);
            driveCallback(desiredSpeeds, true);
            Tracer.endTrace();
        })
            .until(() -> repulsor.atGoal(0.017))
            .withName("PathfindCmd");
    }

    @Override
    public void periodic() {
        Tracer.trace("Drivetrain Data Refresh", () -> {
            OdoThread.getInstance().updatesLock.lock(); // Prevents odometry updates while reading data
            gyro.refreshData(gyroInputs);
            Logger.processInputs("SwerveDrive/Gyro", gyroInputs);
            for (var module : swerveModules) module.periodic();
            OdoThread.getInstance().updatesLock.unlock();
        });

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module: swerveModules) module.stop();
            setpoint = NULL_SETPOINT;
        }

        Tracer.trace("Odometry Update", () -> {
            double[] sampleTimestamps = swerveModules[0].getOdoTimestamps(); // All signals are sampled together
            int sampleCount = sampleTimestamps.length;
            for (int i = 0; i < sampleCount; i++) {
                try {
                    // Read wheel positions and deltas from each module
                    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
                    SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
                    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                        modulePositions[moduleIndex] = swerveModules[moduleIndex].getOdometryFrames()[i];
                        moduleDeltas[moduleIndex] = new SwerveModulePosition(
                            modulePositions[moduleIndex].distanceMeters - measuredModulePositions[moduleIndex].distanceMeters,
                            modulePositions[moduleIndex].angle);
                        measuredModulePositions[moduleIndex] = modulePositions[moduleIndex];
                    }

                    // Update gyro angle
                    if (gyroInputs.connected) {
                        // Use the real gyro angle
                        rawGyroRotation = gyroInputs.odoYawValues[i];
                    } else {
                        // Use the angle delta from the kinematics and module deltas
                        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
                    }

                    // Apply update
                    poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
                } catch (ArrayIndexOutOfBoundsException e) {
                    System.out.println("Array out of bounds");
                }
            }
        });
    }

    @AutoLogOutput
    public ChassisSpeeds robotRelativeSpeeds() {
        var states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) states[i] = swerveModules[i].getState();
        return kinematics.toChassisSpeeds(states);
    }

    @AutoLogOutput
    public double overallSpeedMps() {
        var speeds = robotRelativeSpeeds();
        return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    public void addVisionData(PoseEstimate estimate) {
        poseEstimator.addVisionMeasurement(estimate.pose(), estimate.timestampSecs(), estimate.stdDevs());
    }

    /** Measures the robot's wheel radius by spinning in a circle. */
    public Command wheelRadiusCharacterization() {
        var limiter = new SlewRateLimiter(0.05);
        var state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
            // Drive control sequence
            Commands.sequence(
                // Reset acceleration limiter
                Commands.runOnce(() -> limiter.reset(0.0)),

                // Turn in place, accelerating up to full speed
                this.run(() -> driveCallback(
                    new ChassisSpeeds(0, 0, limiter.calculate(0.25)),
                    false
                ))
            ),
            // Measurement sequence
            Commands.sequence(
                // Wait for modules to fully orient before starting measurement
                Commands.waitSeconds(1.0),
                // Record starting measurement
                Commands.runOnce(
                    () -> {
                        for (int i = 0; i < 4; i++) {
                            state.positions[i] = swerveModules[i].getDistTraveledAngular();
                        }
                        state.lastAngle = gyroInputs.yaw;
                        state.gyroDelta = 0.0;
                    }),

                // Update gyro delta
                Commands.run(() -> {
                    state.gyroDelta += Math.abs(gyroInputs.yaw.minus(state.lastAngle).getRadians());
                    state.lastAngle = gyroInputs.yaw;

                    double[] positions = new double[4];
                    for (int i = 0; i < 4; i++) {
                        positions[i] = swerveModules[i].getDistTraveledAngular();
                    }
                    double wheelDelta = 0.0;
                    for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                    }
                    double wheelRadius =
                        (state.gyroDelta * DRIVEBASE_RADIUS.in(Meters)) / wheelDelta;

                    Logger.recordOutput("wheelRadiusCharacterization/delta", wheelDelta);
                    Logger.recordOutput("wheelRadiusCharacterization/found wheel radius(meters)", wheelRadius);
                })
                    .finallyDo(() -> {
                        double[] positions = new double[4];
                        for (int i = 0; i < 4; i++) {
                            positions[i] = swerveModules[i].getDistTraveledAngular();
                        }
                        double wheelDelta = 0.0;
                        for (int i = 0; i < 4; i++) {
                            wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                        }
                        double wheelRadius =
                            (state.gyroDelta * DRIVEBASE_RADIUS.in(Meters)) / wheelDelta;

                        NumberFormat formatter = new DecimalFormat("#0.000000000000000000000000000");
                        System.out.println(
                            "********** Wheel Radius Characterization Results **********");
                        System.out.println(
                            "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                        System.out.println(
                            "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                        System.out.println(
                            "\tWheel Radius: "
                                + formatter.format(wheelRadius)
                                + " meters, "
                                + formatter.format(metersToInches(wheelRadius))
                                + " inches");
                    })));
    }


    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = Rotation2d.kZero;
        double gyroDelta = 0.0;
    }
}
