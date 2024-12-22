package frc.robot.subsystems.swerve;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.util.AllianceFlipUtil;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.chargers.hardware.encoders.Encoder;
import frc.chargers.hardware.encoders.VoidEncoder;
import frc.chargers.hardware.motorcontrol.Motor;
import frc.chargers.hardware.motorcontrol.SimMotor;
import frc.chargers.utils.InputStream;
import frc.chargers.utils.UtilExtensionMethods;
import frc.chargers.utils.commands.SimpleFeedforwardCharacterization;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.jetbrains.annotations.Nullable;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;
import java.util.function.Supplier;

import static edu.wpi.first.math.MathUtil.angleModulus;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj.DriverStation.Alliance;
import static frc.chargers.utils.UtilMethods.pidControllerFrom;
import static org.photonvision.PhotonUtils.getYawToPose;

/**
 * A drivetrain with 4 drive motors and 4 turn motors.
 * Each turn motor can control the exact position of each drive motor,
 * allowing for omnidirectional movement and driving while turning.
 */
@SuppressWarnings("unused")
@ExtensionMethod(UtilExtensionMethods.class)
public class SwerveDrive extends SubsystemBase {
	public record SwerveDriveConfig(
		HardwareConfig ofHardware,
		ModuleType ofModules,
		ControlsConfig ofControls,
		Supplier<Rotation2d> getRealGyroAngle,
		Function<Double, List<? extends Motor>> getRealDriveMotors,
		Function<Double, List<? extends Motor>> getRealSteerMotors,
		List<Encoder> absoluteEncoders,
		@Nullable TalonFXConfiguration simDriveConfig,
		@Nullable TalonFXConfiguration simSteerConfig
	){}
	
	/**
	 * Use DEFAULT_NEOPRENE_TREAD.cof or COLSONS.cof for coefficientOfFriction,
	 * depending on wheel type.
	 */
	public record HardwareConfig(
		Distance trackWidth,
		Distance wheelBase,
		DCMotor driveMotorType,
		DCMotor turnMotorType,
		LinearVelocity maxVelocity,
		double coefficientOfFriction,
		Mass robotMass
	){}
	
	/**
	 * <h2>IMPORTANT: The driveCurrentLimit is for calculation purposes.
	 * You still must configure your talon fx/spark max to the appropriate limit yourself.
	 */
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
	private final SwerveDrivePoseEstimator poseEstimator;
	private final SwerveDriveSimulation mapleSim;
	@Getter private final SwerveModule[] swerveModules = new SwerveModule[4];
	
	@Setter private boolean useExactVelocityInTeleop = false;
	@Getter @Logged private final SwerveModuleState[] measuredModuleStates = new SwerveModuleState[4];
	private final SwerveModulePosition[] measuredModulePositions = new SwerveModulePosition[4];
	
	@Logged private final PIDController xPoseController;
	@Logged private final PIDController yPoseController;
	@Logged private final PIDController rotationController;
	
	private final HashMap<String, PhotonPoseEstimator> visionEstimators = new HashMap<>();
	private final Supplier<Rotation2d> getAngleFn;
	private Rotation2d prevHeadingCache = Rotation2d.kZero;
	private Pose2d estimatedPose = Pose2d.kZero;
	private final double[] stdDevStore = new double[3];
	
	// workaround before array logging comes around
	@Logged private final SwerveModule topLeftModule;
	@Logged private final SwerveModule topRightModule;
	@Logged private final SwerveModule bottomLeftModule;
	@Logged private final SwerveModule bottomRightModule;
	
	public SwerveDrive(String name, SwerveDriveConfig config) {
		Arrays.fill(measuredModuleStates, new SwerveModuleState());
		Arrays.fill(measuredModulePositions, new SwerveModulePosition());
		
		this.config = config;
		this.name = name;
		this.kinematics = new SwerveDriveKinematics(
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
		);
		this.poseEstimator = new SwerveDrivePoseEstimator(
			kinematics, Rotation2d.kZero, measuredModulePositions, new Pose2d()
		);
		
		var driveSimConfig =
			DriveTrainSimulationConfig.Default()
				.withGyro(COTS.ofPigeon2())
				.withTrackLengthTrackWidth(config.ofHardware.trackWidth, config.ofHardware.trackWidth)
				.withRobotMass(config.ofHardware.robotMass)
				.withBumperSize(
					config.ofHardware.trackWidth.plus(Inches.of(3)),
					config.ofHardware.wheelBase.plus(Inches.of(3))
				);
		
		switch (config.ofModules) {
			case MK4iL2, MK4iL3 -> driveSimConfig.withSwerveModule(
				COTS.ofMark4i(
					config.ofHardware.driveMotorType,
					config.ofHardware.turnMotorType,
					config.ofHardware.coefficientOfFriction,
					config.ofModules == ModuleType.MK4iL3 ? 3 : 2
				)
			);
			
			case SwerveX2SL2, SwerveX2SL3 -> driveSimConfig.withSwerveModule(
				COTS.ofSwerveX2S(
					config.ofHardware.driveMotorType,
					config.ofHardware().turnMotorType,
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
				var steerMotor = new SimMotor(config.ofHardware.turnMotorType, config.ofModules.steerGearRatio, KilogramSquareMeters.of(0.004));
				var driveMotor = new SimMotor(config.ofHardware.driveMotorType, config.ofModules.driveGearRatio, KilogramSquareMeters.of(0.025));
				steerMotor.setPositionPID(config.ofControls.azimuthPID);
				driveMotor.setVelocityPID(config.ofControls.velocityPID);
				steerMotor.enableContinuousInput();
				if (config.simDriveConfig != null) driveMotor.configure(config.simDriveConfig);
				if (config.simSteerConfig != null) steerMotor.configure(config.simSteerConfig);
				this.swerveModules[i] = new SwerveModule(
					config, new VoidEncoder(), steerMotor,
					driveMotor, Optional.of(mapleSim.getModules()[i])
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
				this.swerveModules[i] = new SwerveModule(
					config,
					config.absoluteEncoders.get(i),
					realDriveMotors.get(i),
					realSteerMotors.get(i),
					Optional.empty()
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
	
	private void choreoCallback(SwerveSample trajSample) {
		var vx = trajSample.vx + xPoseController.calculate(getPose().getX(), trajSample.x);
		var vy = trajSample.vy + yPoseController.calculate(getPose().getY(), trajSample.y);
		var rotationV = trajSample.omega + rotationController.calculate(
			angleModulus(getPose().getRotation().getRadians()),
			angleModulus(trajSample.heading)
		);
		driveCallback(
			// do not apply alliance offset; choreo already flips it for you
			ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rotationV, getPose().getRotation()),
			true
		);
	}
	
	public AutoFactory createAutoFactory() {
		return new AutoFactory(
			this::getPose,
			this::resetPose,
			this::choreoCallback,
			true,
			this,
			new AutoFactory.AutoBindings(),
			(trajectory, isStart) -> {
				DogLog.log(name + "/currentTrajectory/samples", trajectory.samples().toArray(new SwerveSample[0]));
				DogLog.log(name + "/currentTrajectory/name", trajectory.name());
			}
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
	
	public SwerveDriveSimulation mapleSimApi() {
		return mapleSim;
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
	
	@Logged public ChassisSpeeds getMeasuredSpeeds() {
		var speeds = getMeasuredSpeedsRobotRelative();
		speeds = ChassisSpeeds.fromRobotRelativeSpeeds(
			speeds,
			addAllianceOffset(getPose().getRotation())
		);
		return speeds;
	}
	
	@Logged public ChassisSpeeds getMeasuredSpeedsRobotRelative() {
		return kinematics.toChassisSpeeds(measuredModuleStates);
	}
	
	// closed loop allows for the robot to drive at an exact velocity.
	private void driveCallback(ChassisSpeeds speeds, boolean closedLoop) {
		speeds = ChassisSpeeds.discretize(speeds, 0.02);
		DogLog.log(name + "/desiredSpeeds", speeds);
		
		var desiredStates = kinematics.toSwerveModuleStates(speeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, config.ofHardware.maxVelocity);
		DogLog.log(name + "/desiredStates", desiredStates);
		for (int i = 0; i < 4; i++) {
			swerveModules[i].setDesiredState(desiredStates[i], closedLoop);
		}
	}
	
	/**
	 * Creates a Command that, when scheduled,
	 * drives the robot forever at the requested x, y, and rotation powers.
	 * Does not move the robot at an exact velocity; so best used in teleop
	 * where exact velocities are not important.
	 */
	public Command driveCmd(
		InputStream getXPower,
		InputStream getYPower,
		InputStream getRotationPower,
		boolean fieldRelative
	) {
		var maxSpeedMps = config.ofHardware.maxVelocity.in(MetersPerSecond);
		return this.run(() -> driveCallback(
			ChassisSpeeds.fromFieldRelativeSpeeds(
				getXPower.get() * maxSpeedMps,
				getYPower.get() * maxSpeedMps,
				getRotationPower.get() * maxSpeedMps,
				fieldRelative ? addAllianceOffset(getPose().getRotation()) : Rotation2d.kZero
			),
			useExactVelocityInTeleop
		)).withName("SwerveDriveCmd" + (useExactVelocityInTeleop ? "(Closed Loop)" : "(Open Loop)"));
	}
	
	/** Creates a Command that, when scheduled, drives the robot to a certain pose. */
	public Command pathFindCmd(Pose2d targetPose, PathConstraints constraints) {
		return new PathfindingCommand(
			targetPose, constraints,
			this::getPose,
			this::getMeasuredSpeedsRobotRelative,
			(speeds, ignore) -> driveCallback(speeds, true),
			new PPHolonomicDriveController(
				config.ofControls.pathTranslationPID,
				config.ofControls.pathRotationPID
			),
			new RobotConfig(
				config.ofHardware.robotMass,
				KilogramSquareMeters.of(6.883), // this value isnt needed for pathfinding
				null,
				kinematics.getModules()
			),
			this
		)
	     // getDistance is an extension method
         .until(() -> getPose().getDistance(targetPose) < 0.5)
         .andThen(
			 this.run(() -> driveCallback(
				 new ChassisSpeeds(
					 xPoseController.calculate(getPose().getX(), targetPose.getX()),
					 yPoseController.calculate(getPose().getY(), targetPose.getY()),
					 rotationController.calculate(
						 getPose().getRotation().getRadians(),
						 targetPose.getRotation().getRadians()
					 )
				 ), true
			 )).until(() -> getPose().getDistance(targetPose) < 0.1)
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
	 * Command cmd = drivetrain.driveCmd(
	 *      controller::getLeftX,
	 *      controller::getLeftY,
	 *      drivetrain.headingLockInputStream(new Pose2d(...), true)
	 * );
	 * cmd.schedule();
	 */
	public InputStream headingLockInputStream(Pose2d pose, boolean shouldFlip) {
		var actualPose = new AtomicReference<Pose2d>();
		return () -> {
			if (actualPose.get() == null) {
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
