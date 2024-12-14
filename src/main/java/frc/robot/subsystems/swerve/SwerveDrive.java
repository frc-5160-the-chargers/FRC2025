package frc.robot.subsystems.swerve;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.util.AllianceFlipUtil;
import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.chargers.hardware.encoders.EncoderIO;
import frc.chargers.hardware.motorcontrol.MotorIO;
import frc.chargers.utils.InputStream;
import frc.chargers.utils.SwerveSetpointGenerator;
import frc.chargers.utils.UtilExtensionMethods;
import frc.chargers.utils.commands.SimpleFeedforwardCharacterization;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.With;
import lombok.experimental.ExtensionMethod;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;
import java.util.function.Supplier;

import static edu.wpi.first.epilogue.Logged.Strategy.OPT_IN;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj.DriverStation.Alliance;
import static frc.chargers.utils.SwerveSetpointGenerator.*;
import static frc.chargers.utils.UtilMethods.pidControllerFrom;
import static org.photonvision.PhotonUtils.getYawToPose;

/**
 * A drivetrain with 4 drive motors and 4 turn motors.
 * Each turn motor can control the exact position of each drive motor,
 * allowing for omnidirectional movement and driving while turning.
 */
@Logged(strategy = OPT_IN)
@SuppressWarnings("unused")
@ExtensionMethod(UtilExtensionMethods.class)
public class SwerveDrive extends SubsystemBase {
	public record SwerveDriveConfig(
		HardwareConfig ofHardware,
		ModuleType ofModules,
		ControlsConfig ofControls,
		ModuleSpeedLimits speedLimits,
		Supplier<Rotation2d> getRealGyroAngle,
		Function<Double, List<MotorIO>> getRealDriveMotors,
		Function<Double, List<MotorIO>> getRealSteerMotors,
		List<EncoderIO> absoluteEncoders
	){}
	
	/**
	 * Use DEFAULT_NEOPRENE_TREAD.cof or COLSONS.cof for coefficientOfFriction,
	 * depending on wheel type.
	 */
	public record HardwareConfig(
		Distance trackWidth,
		Distance wheelBase,
		Distance bumperWidth,
		DCMotor driveMotorType,
		DCMotor turnMotorType,
		double coefficientOfFriction,
		Mass robotMass
	){}
	
	@With
	public record ModuleSpeedLimits(
		LinearVelocity maxVelocity,
		LinearAcceleration maxAccel,
		AngularVelocity maxSteerVelocity
	){}
	
	public record ControlsConfig(
		PIDConstants azimuthPID,
		PIDConstants velocityPID,
		SimpleMotorFeedforward velocityFeedforward,
		PIDConstants pathTranslationPID,
		PIDConstants pathRotationPID
	){}
	
	/** Use ModuleType.MK4iL2 and ModuleType.Mk4iL3. */
	@RequiredArgsConstructor
	public enum ModuleType {
		MK4iL2(6.75, 150.0 / 7.0, Inches.of(2)),
		MK4iL3(6.12, 150.0 / 7.0, Inches.of(2)),
		SwerveX2SL2(1.0, 1.0, Inches.of(0)),
		SwerveX2SL3(1.0, 1.0, Inches.of(0)); // TODO
		
		public final double driveGearRatio;
		public final double steerGearRatio;
		public final Distance wheelRadius;
	}
	
	private final String name;
	private final SwerveDriveConfig config;
	
	private final SwerveDriveKinematics kinematics;
	private final SwerveSetpointGenerator setpointGen;
	private final SwerveDrivePoseEstimator poseEstimator;
	private final SwerveDriveSimulation mapleSim;
	private final BaseSwerveModule[] swerveModules = new BaseSwerveModule[4];
	
	@Getter @Logged private final SwerveModuleState[] measuredModuleStates = new SwerveModuleState[4];
	private final SwerveModulePosition[] measuredModulePositions = new SwerveModulePosition[4];
	private SwerveSetpoint desiredState = new SwerveSetpoint();
	private ModuleSpeedLimits moduleLimits;
	
	@Logged private final PIDController xPoseController;
	@Logged private final PIDController yPoseController;
	@Logged private final PIDController rotationController;
	
	private final HashMap<String, PhotonPoseEstimator> visionEstimators = new HashMap<>();
	private final Supplier<Rotation2d> getAngleFn;
	private Rotation2d prevHeadingCache = Rotation2d.kZero;
	private Pose2d estimatedPose = Pose2d.kZero;
	private final double[] stdDevStore = new double[3];
	
	// workaround before array logging comes around
	@Logged private final BaseSwerveModule topLeftModule;
	@Logged private final BaseSwerveModule topRightModule;
	@Logged private final BaseSwerveModule bottomLeftModule;
	@Logged private final BaseSwerveModule bottomRightModule;
	
	public SwerveDrive(String name, SwerveDriveConfig config) {
		Arrays.fill(measuredModuleStates, new SwerveModuleState());
		Arrays.fill(measuredModulePositions, new SwerveModulePosition());
		
		this.config = config;
		this.name = name;
		Translation2d[] moduleLocations = {
			new Translation2d(
				config.ofHardware.trackWidth.div(2),
				config.ofHardware.wheelBase.div(2)
			),
			new Translation2d(
				config.ofHardware.trackWidth.div(2),
				config.ofHardware.wheelBase.div(-2)
			),
			new Translation2d(
				config.ofHardware.trackWidth.div(-2),
				config.ofHardware.wheelBase.div(2)
			),
			new Translation2d(
				config.ofHardware.trackWidth.div(-2),
				config.ofHardware.wheelBase.div(-2)
			)
		};
		this.kinematics = new SwerveDriveKinematics(moduleLocations);
		this.setpointGen = new SwerveSetpointGenerator(kinematics, moduleLocations);
		this.poseEstimator = new SwerveDrivePoseEstimator(
			kinematics, Rotation2d.kZero, measuredModulePositions, new Pose2d()
		);
		this.moduleLimits = config.speedLimits;
		
		var driveSimConfig =
			DriveTrainSimulationConfig.Default()
				.withGyro(GyroSimulation.getNav2X())
				.withTrackLengthTrackWidth(config.ofHardware.trackWidth, config.ofHardware.trackWidth)
				.withBumperSize(
					config.ofHardware.trackWidth.plus(config.ofHardware.bumperWidth),
					config.ofHardware.wheelBase.plus(config.ofHardware.bumperWidth)
				);
		
		switch (config.ofModules) {
			case MK4iL2, MK4iL3 -> driveSimConfig.withSwerveModule(
				SwerveModuleSimulation.getMark4i(
					config.ofHardware.driveMotorType,
					config.ofHardware().turnMotorType,
					Amps.of(60),
					config.ofHardware.coefficientOfFriction,
					config.ofModules == ModuleType.MK4iL3 ? 3 : 2
				)
			);
			
			case SwerveX2SL2, SwerveX2SL3 -> driveSimConfig.withSwerveModule(
				SwerveModuleSimulation.getSwerveX2S(
					config.ofHardware.driveMotorType,
					config.ofHardware().turnMotorType,
					Amps.of(60),
					config.ofHardware.coefficientOfFriction,
					config.ofModules == ModuleType.SwerveX2SL3 ? 3 : 2
				)
			);
		}
		
		this.mapleSim = new SwerveDriveSimulation(driveSimConfig, new Pose2d());
		this.xPoseController = pidControllerFrom(config.ofControls.pathTranslationPID);
		this.yPoseController = pidControllerFrom(config.ofControls.pathTranslationPID);
		this.rotationController = pidControllerFrom(config.ofControls.pathRotationPID);
		this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
		
		if (RobotBase.isSimulation()) {
			for (int i = 0; i < 4; i++) {
				this.swerveModules[i] = new SimSwerveModule(
					mapleSim.getModules()[i], config.ofModules.wheelRadius, config.speedLimits.maxVelocity
				);
			}
			this.getAngleFn = mapleSim.getGyroSimulation()::getGyroReading;
			SimulatedArena.getInstance().addDriveTrainSimulation(mapleSim);
		} else {
			var realDriveMotors = config.getRealDriveMotors.apply(config.ofModules.driveGearRatio);
			var realSteerMotors = config.getRealSteerMotors.apply(config.ofModules.steerGearRatio);
			for (int i = 0; i < 4; i++) {
				realDriveMotors.get(i).setPositionPID(config.ofControls.azimuthPID);
				realDriveMotors.get(i).setVelocityPID(config.ofControls.velocityPID);
				realSteerMotors.get(i).enableContinuousInput();
				
				this.swerveModules[i] = new RealSwerveModule(
					realDriveMotors.get(i),
					realSteerMotors.get(i),
					config.absoluteEncoders.get(i),
					config.ofModules.wheelRadius,
					config.speedLimits.maxVelocity,
					config.ofControls.velocityFeedforward
				);
			}
			this.getAngleFn = config.getRealGyroAngle;
		}
		this.topLeftModule = this.swerveModules[0];
		this.topRightModule = this.swerveModules[1];
		this.bottomLeftModule = this.swerveModules[2];
		this.bottomRightModule = this.swerveModules[3];
	}
	
	private boolean isRedAlliance() {
		return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
	}
	
	private Rotation2d addAllianceOffset(Rotation2d base) {
		return isRedAlliance() ? base.plus(Rotation2d.k180deg) : base;
	}
	
	private void logTrajectory(Trajectory<SwerveSample> traj, boolean isStart) {
		if (isRedAlliance()) traj = traj.flipped();
		if (isStart) {
			DogLog.log(name + "/currentTrajectory/poses", traj.getPoses());
			DogLog.log(name + "/currentTrajectory/totalTime", traj.getTotalTime());
			DogLog.log(name + "/currentTrajectory/name", traj.name());
		}
	}
	
	private void choreoCallback(SwerveSample trajSample) {
		var vx = trajSample.vx + xPoseController.calculate(getPose().getX(), trajSample.x);
		var vy = trajSample.vy + yPoseController.calculate(getPose().getY(), trajSample.y);
		var rotationV = trajSample.omega + rotationController.calculate(
			getPose().getRotation().getRadians(),
			trajSample.heading
		);
		// closed loop, not field relative
		driveCallback(new ChassisSpeeds(vx, vy, rotationV), true, false);
	}
	
	public AutoFactory createAutoFactory() {
		return Choreo.createAutoFactory(
			this::getPose,
			this::choreoCallback,
			this::isRedAlliance,
			this,
			new AutoFactory.AutoBindings(),
			this::logTrajectory
		);
	}
	
	@Logged
	public Pose2d getPose() {
		if (RobotBase.isSimulation()) {
			return mapleSim.getSimulatedDriveTrainPose();
		} else {
			return estimatedPose;
		}
	}
	
	public void resetPose(Pose2d pose) {
		if (RobotBase.isSimulation()) mapleSim.setSimulationWorldPose(pose);
		poseEstimator.resetPose(pose);
	}
	
	public void addVisionCamera(
		PhotonCamera cam,
		Transform3d robotToCamera,
		PhotonPoseEstimator.PoseStrategy strategy
	) {
		visionEstimators.put(
			cam.getName(),
			new PhotonPoseEstimator(
				// TODO change to reefscape
				AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo),
				strategy, cam, robotToCamera
			)
		);
	}
	
	public void setMaxAccel(LinearAcceleration accel) {
		moduleLimits = moduleLimits.withMaxAccel(accel);
	}
	
	@Logged public ChassisSpeeds getMeasuredSpeeds() {
		var speeds = getMeasuredSpeedsRobotRelative();
		// TODO change this once ChassisSpeeds becomes immutable
		speeds.toFieldRelativeSpeeds(
			addAllianceOffset(getPose().getRotation())
		);
		return speeds;
	}
	
	@Logged public ChassisSpeeds getMeasuredSpeedsRobotRelative() {
		return kinematics.toChassisSpeeds(measuredModuleStates);
	}
	
	// closed loop allows for the robot to drive at an exact velocity.
	private void driveCallback(ChassisSpeeds speeds, boolean closedLoop, boolean fieldRelative) {
		// TODO change this when ChassisSpeeds becomes immutable
		if (fieldRelative) {
			speeds.toRobotRelativeSpeeds(
				addAllianceOffset(getPose().getRotation())
			);
		}
		speeds.discretize(0.04);
		desiredState = setpointGen.generateSetpoint(
			moduleLimits.maxVelocity,
			moduleLimits.maxAccel,
			moduleLimits.maxSteerVelocity,
			desiredState, speeds, 0.02
		);
		DogLog.log(name + "/desiredModuleStates", desiredState.moduleStates());
		DogLog.log(name + "/desiredSpeeds(final)", desiredState.chassisSpeeds());
		for (int i = 0; i < 4; i++) {
			swerveModules[i].setDesiredState(desiredState.moduleStates()[i], closedLoop);
		}
	}
	
	/**
	 * Drives the robot forever at the requested x, y, and rotation powers.
	 * Does not move the robot at an exact velocity; so best used in teleop
	 * where exact velocities are not important.
	 */
	public Command teleopDriveCmd(
		InputStream getXPower,
		InputStream getYPower,
		InputStream getRotationPower,
		boolean fieldRelative
	) {
		var maxSpeedMps = config.speedLimits.maxVelocity.in(MetersPerSecond);
		return this.run(() -> driveCallback(
			new ChassisSpeeds(
				getXPower.get() * maxSpeedMps,
				getYPower.get() * maxSpeedMps,
				getRotationPower.get() * maxSpeedMps
			),
			false, fieldRelative
		)).withName("SwerveDriveCmd(open loop)");
	}
	
	/** Drives the robot forever at the exact chassis speeds requested. */
	public Command velocityDriveCmd(Supplier<ChassisSpeeds> getChassisSpeeds, boolean fieldRelative) {
		return this.run(() -> driveCallback(getChassisSpeeds.get(), true, fieldRelative))
			       .withName("SwerveDriveCmd(closed loop)");
	}
	
	public Command pathFindCmd(Pose2d targetPose, PathConstraints constraints) {
		return new PathfindHolonomic(
			targetPose,
			constraints,
			this::getPose,
			this::getMeasuredSpeedsRobotRelative,
			speeds -> driveCallback(speeds, true, false),
			new HolonomicPathFollowerConfig(
				config.speedLimits.maxVelocity.in(MetersPerSecond),
				config.ofHardware.wheelBase.in(Meters) / 2,
				new ReplanningConfig()
			)
		).until(() -> getPose().minus(targetPose).getTranslation().getNorm() < 0.7)
         .andThen(
			 this.run(() -> driveCallback(
				 new ChassisSpeeds(
					 xPoseController.calculate(getPose().getX(), targetPose.getX()),
					 yPoseController.calculate(getPose().getY(), targetPose.getY()),
					 rotationController.calculate(
						 getPose().getRotation().getRadians(),
						 targetPose.getRotation().getRadians()
					 )
				 ), true, false
			 )).until(() -> getPose().getDistance(targetPose) < 0.1) // distanceTo is an extension method
         )
         .withName("SwervePathfindCmd");
	}
	
	/**
	 * Creates a rotation input stream that locks the robot's heading.
	 * Example:
	 * <pre><code>
	 * Command cmd = drivetrain.teleopDriveCmd(
	 *      controller::getLeftX,
	 *      controller::getLeftY,
	 *      drivetrain.headingLockInputStream(Radians.of(2.0))
	 * );
	 * cmd.schedule();
	 */
	public InputStream headingLockInputStream(Angle targetHeading) {
		return () -> {
			double output = rotationController.calculate(
				getPose().getRotation().getRadians(),
				targetHeading.in(Radians)
			);
			DogLog.log(name + "/headingLockOutput", output);
			return output;
		};
	}
	
	/**
	 * Creates a rotation input stream that locks the robot's heading.
	 * Example:
	 * <pre><code>
	 * Command cmd = drivetrain.teleopDriveCmd(
	 *      controller::getLeftX,
	 *      controller::getLeftY,
	 *      drivetrain.headingLockInputStream(new Pose2d(...), true)
	 * );
	 * cmd.schedule();
	 */
	public InputStream headingLockInputStream(Pose2d pose, boolean shouldFlip) {
		var actualPose = new AtomicReference<Pose2d>();
		return () -> {
			if (actualPose.get() != null) {
				actualPose.set(shouldFlip ? AllianceFlipUtil.flip(pose) : pose);
			}
			double output = rotationController.calculate(
				getPose().getRotation().getRadians(),
				getYawToPose(getPose(), actualPose.get()).getRadians()
			);
			DogLog.log(name + "/headingLockOutput", output);
			return output;
		};
	}
	
	/**
	 * Runs feedforward characterization forever until interrupt.
	 * On end, prints feedforward numbers.
	 */
	public Command characterizeFeedforwardCmd() {
		return new SimpleFeedforwardCharacterization(
			this,
			volts -> {
				for (var module: swerveModules) {
					module.setDriveVoltage(volts);
					module.setSteerVoltage(0.0);
				}
			},
			() -> {
				double totalVel = 0.0;
				for (var module: swerveModules) { totalVel += module.currentState().speedMetersPerSecond; }
				return totalVel / 4.0;
			}
		).withName("SwerveCharacterizeCmd");
	}
 
	@Override
	public void periodic() {
		Rotation2d latestHeading = getAngleFn.get();
		for (int i = 0; i < 4; i++) {
			measuredModuleStates[i] = swerveModules[i].currentState();
			measuredModulePositions[i] = swerveModules[i].currentPosition();
			swerveModules[i].periodic();
			if (DriverStation.isDisabled()) {
				swerveModules[i].setDriveVoltage(0.0);
				swerveModules[i].setSteerVoltage(0.0);
			}
		}
		
		// if rotating too fast, discard vision measurements
		if (Math.abs(latestHeading.minus(prevHeadingCache).getDegrees() / 0.02) < 720) {
			for (var name: visionEstimators.keySet()) {
				var yawOfVisionEstimator =
					visionEstimators.get(name)
						.getRobotToCameraTransform()
						.getRotation()
						.getZ();
				var data = visionEstimators.get(name).update();
				if (data.isPresent()) {
					var stdDevs = calculateVisionUncertainty(
						getPose().getX(), latestHeading, Rotation2d.fromRadians(yawOfVisionEstimator)
					);
					DogLog.log("vision/cameras/" + name + "/hasPose", true);
					DogLog.log("vision/cameras/" + name + "/pose", data.get().estimatedPose);
					for (int i = 0; i < 3; i++) stdDevStore[i] = stdDevs.get(i, 0);
					DogLog.log("vision/cameras/" + name + "/stdDevs", stdDevStore);
					poseEstimator.addVisionMeasurement(
						data.get().estimatedPose.toPose2d(),
						data.get().timestampSeconds,
						stdDevs
					);
				} else {
					DogLog.log("vision/cameras/" + name + "/hasPose", false);
					DogLog.log("vision/cameras/" + name + "/pose", Pose3d.kZero);
				}
			}
		}
		
		estimatedPose = poseEstimator.update(latestHeading, measuredModulePositions);
		prevHeadingCache = latestHeading;
		
		// on the real robot, logged pose = computed pose
		if (RobotBase.isSimulation()) DogLog.log(name + "/odometryPose", estimatedPose);
	}
	
	// Credits: 6995
	private static Matrix<N3, N1> calculateVisionUncertainty(
		double poseX, Rotation2d heading, Rotation2d cameraYaw
	) {
		double maximumUncertainty = 3;
		double minimumUncertainty = 0.1;
		double a = 6;
		double b = -1.3;
		Rotation2d cameraWorldYaw = cameraYaw.rotateBy(heading);
		boolean isCameraFacingFieldSideTags;
		boolean facingRedAlliance;
		double distanceFromTagSide;
		
		if (-90 < cameraWorldYaw.getDegrees() && cameraWorldYaw.getDegrees() < 90) {
			// camera facing towards red alliance
			facingRedAlliance = true;
			isCameraFacingFieldSideTags = poseX > 16.5 / 2;
		} else {
			// camera facing towards blue alliance
			facingRedAlliance = false;
			isCameraFacingFieldSideTags = poseX < 16.5 / 2;
		}
		
		if (isCameraFacingFieldSideTags) {
			// uncertainty low
			if (facingRedAlliance) {
				distanceFromTagSide = 16.5 - poseX;
			} else {
				distanceFromTagSide = poseX;
			}
		} else {
			// uncertainty high
			if (!facingRedAlliance) {
				distanceFromTagSide = poseX;
			} else {
				distanceFromTagSide = 16.5 - poseX;
			}
		}
		double positionUncertainty =
			((maximumUncertainty - minimumUncertainty)
				 / (1 + Math.pow(Math.E, (a + (b * distanceFromTagSide)))))
				+ minimumUncertainty;
		return VecBuilder.fill(positionUncertainty, positionUncertainty, 10000);
	}
}
