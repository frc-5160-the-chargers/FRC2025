package frc.robot.subsystems.swerve;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.chargers.hardware.encoders.EncoderIO;
import frc.chargers.hardware.motorcontrol.MotorIO;
import frc.chargers.utils.InputStream;
import frc.chargers.utils.Logger;
import frc.chargers.utils.UtilExtensionMethods;
import frc.chargers.utils.commands.SimpleFeedforwardCharacterization;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
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
import java.util.function.Function;
import java.util.function.Supplier;

import static edu.wpi.first.epilogue.Logged.Strategy.OPT_IN;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj.DriverStation.Alliance;
import static frc.chargers.utils.UtilMethods.pidControllerFrom;

/**
 * A drivetrain with 4 drive motors and 4 turn motors.
 * Each turn motor can control the exact position of each drive motor,
 * allowing for omnidirectional movement and driving while turning.
 */
@SuppressWarnings("unused")
@Logged(strategy = OPT_IN)
@ExtensionMethod(UtilExtensionMethods.class)
public class SwerveDrive extends SubsystemBase {
	public record SwerveDriveConfig(
		HardwareConfig ofHardware,
		ModuleType ofModules,
		ControlsConfig ofControls,
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
		LinearVelocity maxSpeed,
		double coefficientOfFriction,
		Mass robotMass
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
	
	private final SwerveDriveConfig config;
	
	private final SwerveDriveKinematics kinematics;
	private final SwerveDrivePoseEstimator poseEstimator;
	private final SwerveDriveSimulation mapleSim;
	private final BaseSwerveModule[] swerveModules = new BaseSwerveModule[4];
	
	@Logged private final PIDController xPoseController;
	@Logged private final PIDController yPoseController;
	@Logged private final PIDController rotationController;
	private final Field2d simpleFieldViz = new Field2d();
	
	private final HashMap<String, PhotonPoseEstimator> visionEstimators = new HashMap<>();
	@Getter @Logged private final SwerveModuleState[] measuredModuleStates = new SwerveModuleState[4];
	@Getter @Logged private SwerveModuleState[] desiredModuleStates = new SwerveModuleState[4];
	@Getter @Logged private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
	private final SwerveModulePosition[] measuredModulePositions = new SwerveModulePosition[4];
	private final Supplier<Rotation2d> getAngleFn;
	private Rotation2d prevHeadingCache = Rotation2d.kZero;
	private Pose2d latestPose = new Pose2d();
	@Setter private boolean field2dEnabled = false;
	
	// workaround before array logging comes around
	@Logged private final BaseSwerveModule topLeftModule;
	@Logged private final BaseSwerveModule topRightModule;
	@Logged private final BaseSwerveModule bottomLeftModule;
	@Logged private final BaseSwerveModule bottomRightModule;
	
	public SwerveDrive(SwerveDriveConfig config) {
		Arrays.fill(measuredModuleStates, new SwerveModuleState());
		Arrays.fill(desiredModuleStates, new SwerveModuleState());
		Arrays.fill(measuredModulePositions, new SwerveModulePosition());
		
		this.config = config;
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
		SmartDashboard.putData("SwerveDrive/fieldViz", this.simpleFieldViz);
		
		if (RobotBase.isSimulation()) {
			for (int i = 0; i < 4; i++) {
				this.swerveModules[i] = new SimSwerveModule(
					mapleSim.getModules()[i], config.ofModules.wheelRadius, config.ofHardware.maxSpeed
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
					config.ofHardware.maxSpeed,
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
			Logger.log("Choreo/currentTrajectory", traj.getPoses());
			Logger.log("Choreo/currentTrajectoryFinalPose", traj.getFinalPose(false));
		}
	}
	
	public AutoFactory createAutoFactory() {
		return createAutoFactory(new AutoFactory.AutoBindings());
	}
	
	public AutoFactory createAutoFactory(AutoFactory.AutoBindings autoBindings) {
		return Choreo.createAutoFactory(
			this,
			this::getPose,
			(currentPose, trajectorySample) -> driveCallback(
				// closed loop, not field relative
				new ChassisSpeeds(
					xPoseController.calculate(currentPose.getX(), trajectorySample.x),
					yPoseController.calculate(currentPose.getY(), trajectorySample.y),
					rotationController.calculate(currentPose.getRotation().getRadians(), trajectorySample.omega)
				),
				true, false
			),
			this::isRedAlliance,
			autoBindings,
			this::logTrajectory
		);
	}
	
	@Logged
	public Pose2d getPose() {
		if (RobotBase.isSimulation()) {
			return mapleSim.getSimulatedDriveTrainPose();
		} else {
			return latestPose;
		}
	}
	
	public void resetPose(Pose2d pose) {
		if (RobotBase.isSimulation()) {
			mapleSim.setSimulationWorldPose(pose);
		} else {
			poseEstimator.resetPose(pose);
		}
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
		return kinematics.toChassisSpeeds(measuredModuleStates);
	}
	
	@Logged public ChassisSpeeds getMeasuredSpeedsRobotRelative() {
		var speeds = getMeasuredSpeeds();
		// TODO change this once ChassisSpeeds becomes immutable
		speeds.toRobotRelativeSpeeds(
			addAllianceOffset(getPose().getRotation())
		);
		return speeds;
	}
	
	private ChassisSpeeds limitMaxSpeed(ChassisSpeeds speeds) {
		var modStates = kinematics.toSwerveModuleStates(speeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(modStates, config.ofHardware.maxSpeed);
		return kinematics.toChassisSpeeds(modStates);
	}
	
	// closed loop allows for the robot to drive at an exact velocity.
	private void driveCallback(ChassisSpeeds speeds, boolean closedLoop, boolean fieldRelative) {
		// TODO change this when ChassisSpeeds becomes immutable
		if (fieldRelative) {
			speeds.toRobotRelativeSpeeds(
				addAllianceOffset(getPose().getRotation())
			);
		}
		speeds = limitMaxSpeed(speeds);
		speeds.discretize(0.02);
		this.desiredSpeeds = speeds;
		this.desiredModuleStates = kinematics.toSwerveModuleStates(speeds);
		for (int i = 0; i < 4; i++) {
			swerveModules[i].setDesiredState(desiredModuleStates[i], closedLoop);
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
		var maxSpeedMps = config.ofHardware.maxSpeed.in(MetersPerSecond);
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
				config.ofHardware.maxSpeed.in(MetersPerSecond),
				config.ofHardware.wheelBase.in(Meters) / 2,
				new ReplanningConfig()
			)
		).until(() -> getPose().minus(targetPose).getTranslation().getNorm() < 1.0)
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
					Logger.log("vision/cameras/" + name + "/hasPose", true);
					Logger.log("vision/cameras/" + name + "/pose", data.get().estimatedPose);
					poseEstimator.addVisionMeasurement(
						data.get().estimatedPose.toPose2d(),
						data.get().timestampSeconds,
						calculateVisionUncertainty(
							latestPose.getX(), latestHeading, Rotation2d.fromRadians(yawOfVisionEstimator)
						)
					);
				} else {
					Logger.log("vision/cameras/" + name + "/hasPose", false);
					Logger.log("vision/cameras/" + name + "/pose", Pose3d.kZero);
				}
			}
		}
		
		if (RobotBase.isSimulation()) {
			// no vision updates in sim
			if (field2dEnabled) simpleFieldViz.setRobotPose(mapleSim.getSimulatedDriveTrainPose());
		} else {
			latestPose = poseEstimator.update(latestHeading, measuredModulePositions);
			if (field2dEnabled) simpleFieldViz.setRobotPose(latestPose);
			prevHeadingCache = latestHeading;
		}
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
