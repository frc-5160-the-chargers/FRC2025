package frc.robot.subsystems.swerve;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.chargers.hardware.encoders.Encoder;
import frc.chargers.hardware.encoders.VoidEncoder;
import frc.chargers.hardware.motorcontrol.Motor;
import frc.chargers.utils.AllianceFlipper;
import frc.chargers.utils.InputStream;
import frc.chargers.utils.PIDConstants;
import frc.chargers.utils.RepulsorFieldPlanner;
import frc.chargers.utils.UtilMethods;
import frc.robot.subsystems.StandardSubsystem;
import frc.robot.vision.GlobalPoseEstimate;
import frc.robot.vision.SingleTagPoseEstimate;
import frc.robot.vision.SingleTagPoseEstimator;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.experimental.ExtensionMethod;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.jetbrains.annotations.Nullable;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;
import java.util.function.Supplier;

import static edu.wpi.first.math.MathUtil.angleModulus;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * A drivetrain with 4 drive motors and 4 turn motors.
 * Each turn motor can control the exact position of each drive motor,
 * allowing for omnidirectional movement and driving while turning.
 */
@ExtensionMethod(UtilMethods.class)
@SuppressWarnings("unused")
public class SwerveDrive extends StandardSubsystem {
	public record SwerveDriveConfig(
		HardwareConfig ofHardware,
		ModuleType ofModules,
		ControlsConfig ofControls,
		Supplier<Rotation2d> getRealGyroAngle,
		Function<SwerveCorner, Motor> realSteerMotorCreator,
		Function<SwerveCorner, Motor> realDriveMotorCreator,
		Function<SwerveCorner, Encoder> realAbsEncoderCreator,
		@Nullable TalonFXConfiguration simSteerConfig,
		@Nullable TalonFXConfiguration simDriveConfig
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
	){
		public Distance drivebaseRadius() {
			return Meters.of(Math.hypot(trackWidth.in(Meters) / 2, wheelBase.in(Meters) / 2));
		}
	}
	
	public record ControlsConfig(
		PIDConstants steerPID,
		PIDConstants velocityPID,
		SimpleMotorFeedforward velocityFeedforward,
		PIDConstants pathTranslationPID,
		PIDConstants pathRotationPID,
		double forceKT,
		PoseEstimationMode poseEstimationMode
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
	
	public enum SwerveCorner {
		TOP_LEFT, TOP_RIGHT, BOTTOM_LEFT, BOTTOM_RIGHT
	}
	
	public enum PoseEstimationMode {
		AUTOMATIC, SELF_RUN
	}
	
	private static final SwerveSetpoint NULL_SETPOINT = new SwerveSetpoint(
		new ChassisSpeeds(),
		new SwerveModuleState[]{ new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState() },
		DriveFeedforwards.zeros(4)
	);

	@Getter private final SwerveDriveConfig config;
	@Getter private final SwerveDriveKinematics kinematics;
	@Getter private final SwerveModule[] swerveModules = new SwerveModule[4];
	private final SwerveDrivePoseEstimator poseEstimator;
	private final SingleTagPoseEstimator singleTagPoseEstimator;
	private final SwerveDriveSimulation mapleSim;
	private final RepulsorFieldPlanner repulsor = new RepulsorFieldPlanner();
	
	@Logged private boolean acceptVisionObservations = true;
	@Logged private final SwerveModuleState[] measuredModuleStates = new SwerveModuleState[4];
	private final SwerveModulePosition[] measuredModulePositions = new SwerveModulePosition[4];

	private final PIDController xPoseController;
	private final PIDController yPoseController;
	private final PIDController rotationController;

	private final Supplier<Rotation2d> getAngleFn;
	private Rotation2d prevHeadingCache = Rotation2d.kZero;
	private final double[] stdDevStore = new double[3];
	@Logged(name = "singleTagEstimation/id")
	private int targetTagId = -1;
	private boolean useSingleTagEstimation = false;

	// workaround before array logging comes around
	@Logged private final SwerveModule topLeftModule;
	@Logged private final SwerveModule topRightModule;
	@Logged private final SwerveModule bottomLeftModule;
	@Logged private final SwerveModule bottomRightModule;
	
	public SwerveDrive(SwerveDriveConfig config) {
		Arrays.fill(measuredModuleStates, new SwerveModuleState());
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
			kinematics, Rotation2d.kZero, measuredModulePositions, Pose2d.kZero
		);
		this.singleTagPoseEstimator = new SingleTagPoseEstimator(kinematics, VecBuilder.fill(0.003, 0.003, 0.02));

		var driveSimConfig =
			DriveTrainSimulationConfig.Default()
				.withTrackLengthTrackWidth(config.ofHardware.trackWidth, config.ofHardware.trackWidth)
				.withRobotMass(config.ofHardware.robotMass)
				.withGyro(COTS.ofPigeon2())
				.withBumperSize(
					config.ofHardware.trackWidth.plus(Inches.of(6)),
					config.ofHardware.wheelBase.plus(Inches.of(6))
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

		this.mapleSim = new SwerveDriveSimulation(driveSimConfig, Pose2d.kZero);
		this.xPoseController = config.ofControls.pathTranslationPID.asController();
		this.yPoseController = config.ofControls.pathTranslationPID.asController();
		this.rotationController = config.ofControls.pathRotationPID.asController();
		this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
		
		for (int i = 0; i < 4; i++) {
			Motor steerMotor;
			Motor driveMotor;
			Encoder absoluteEncoder;
			if (RobotBase.isSimulation()) {
				var moduleSim = mapleSim.getModules()[i];
				// useSteerMotorController and useDriveMotorController return the passed in DummyMotor.
				steerMotor = moduleSim.useSteerMotorController(new SwerveModule.DummyMotor(config.simSteerConfig));
				driveMotor = moduleSim.useDriveMotorController(new SwerveModule.DummyMotor(config.simDriveConfig));
				absoluteEncoder = new VoidEncoder();
			} else {
				// SwerveCorner.values() is TL, TR, BL, and BR(enums are defined in this order)
				var corner = SwerveCorner.values()[i];
				steerMotor = config.realSteerMotorCreator.apply(corner);
				driveMotor = config.realDriveMotorCreator.apply(corner);
				absoluteEncoder = config.realAbsEncoderCreator.apply(corner);
			}
			
			steerMotor.setControlsConfig(
				Motor.ControlsConfig.EMPTY
					.withGearRatio(config.ofModules.steerGearRatio)
					.withPositionPID(config.ofControls.steerPID)
					.withContinuousInput(true)
			);
			driveMotor.setControlsConfig(
				Motor.ControlsConfig.EMPTY
					.withGearRatio(config.ofModules.driveGearRatio)
					.withVelocityPID(config.ofControls.velocityPID)
			);
			
			this.swerveModules[i] = new SwerveModule(driveMotor, steerMotor, absoluteEncoder, config);
		}
		if (RobotBase.isSimulation()) {
			this.getAngleFn = () -> mapleSim.getSimulatedDriveTrainPose().getRotation();
			SimulatedArena.getInstance().addDriveTrainSimulation(mapleSim);
		} else {
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

	private Rotation2d offsetWithAlliance(Rotation2d base) {
		return isRedAlliance() ? base.plus(Rotation2d.k180deg) : base;
	}
	
	/** Obtains desired module states from a ChassisSpeeds target. */
	private SwerveModuleState[] toDesiredModuleStates(ChassisSpeeds speeds, boolean limitVelocity) {
		speeds = ChassisSpeeds.discretize(speeds, 0.02);
		log("desiredSpeeds", speeds);
		var desiredStates = kinematics.toSwerveModuleStates(speeds);
		if (limitVelocity) SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, config.ofHardware.maxVelocity);
		for (int i = 0; i < 4; i++) {
			var currentAngle = swerveModules[i].getSteerAngle();
			desiredStates[i].optimize(currentAngle);
			desiredStates[i].cosineScale(currentAngle);
		}
		log("desiredStates", desiredStates);
		return desiredStates;
	}
	
	/** Obtains desired module states from a choreo trajectory sample. */
	private ChassisSpeeds toDesiredSpeeds(SwerveSample trajSample) {
		var vx = trajSample.vx + xPoseController.calculate(poseEstimate().getX(), trajSample.x);
		var vy = trajSample.vy + yPoseController.calculate(poseEstimate().getY(), trajSample.y);
		var rotationV = trajSample.omega + rotationController.calculate(
			angleModulus(poseEstimate().getRotation().getRadians()),
			angleModulus(trajSample.heading)
		);
		return ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rotationV, poseEstimate().getRotation());
	}
	
	@Logged private List<SwerveSample> trajSamples = List.of();
	
	/** Creates a choreo AutoFactory. You should cache this in your Robot or AutoCommands class. */
	public AutoFactory createAutoFactory() {
		return new AutoFactory(
			this::poseEstimate,
			this::resetPose,
			// a function that runs trajectory following
			(SwerveSample trajSample) -> {
				var desiredModuleStates = toDesiredModuleStates(toDesiredSpeeds(trajSample), false);
				for (int i = 0; i < 4; i++) {
					var desiredState = desiredModuleStates[i];
					double wheelTorqueNm = config.ofModules.wheelRadius.in(Meters) * (
						desiredState.angle.getCos() * trajSample.moduleForcesX()[i] +
							desiredState.angle.getSin() * trajSample.moduleForcesY()[i]
					);
					double torqueFF = wheelTorqueNm / config.ofModules.driveGearRatio * config.ofControls.forceKT;
					swerveModules[i].setDesiredState(desiredState, true, torqueFF);
				}
			},
			true,
			this,
			(trajectory, isStart) -> {
				trajSamples = trajectory.samples();
				log("currentTrajectory/name", trajectory.name());
			}
		);
	}
	
	/** Creates a swerve setpoint generator. This is primarily used to reduce odometry drift when pathfinding. */
	public SwerveSetpointGenerator createSetpointGenerator(
		MomentOfInertia bodyMoi, Current driveCurrentLimit
	) {
		return new SwerveSetpointGenerator(
			new RobotConfig(
				config.ofHardware.robotMass,
				bodyMoi,
				new ModuleConfig(
					config.ofModules.wheelRadius,
					config.ofHardware.maxVelocity,
					config.ofHardware.coefficientOfFriction,
					config.ofHardware.driveMotorType(),
					config.ofModules.driveGearRatio,
					driveCurrentLimit,
					4
				),
				kinematics.getModules()
			),
			config.ofHardware.turnMotorType.freeSpeedRadPerSec
		);
	}
	
	public void enableSingleTagEstimation(int targetTagId) {
		useSingleTagEstimation = true;
		this.targetTagId = targetTagId;
	}
	
	public void disableSingleTagEstimation() {
		useSingleTagEstimation = false;
	}
	
	/** Gets the drivetrain's calculated pose estimate. */
	@Logged
	public Pose2d poseEstimate() {
		var multiTagEstimate = poseEstimator.getEstimatedPosition();
		if (useSingleTagEstimation) {
			var singleTagEstimate = singleTagPoseEstimator.getEstimatedPosition(targetTagId);
			log("singleTagEstimation/used", singleTagEstimate.isPresent());
			return singleTagPoseEstimator.getEstimatedPosition(targetTagId).orElse(multiTagEstimate);
		} else {
			log("singleTagEstimation/used", false);
			return multiTagEstimate;
		}
	}
	
	/**
	 * In simulation, gets the actual pose of the drivetrain
	 * (ignoring simulated motor drift, collisions, and inaccuracy) - calculated by maple sim
	 * On the real robot, returns the same thing as poseEstimate(). <br />
	 * Note that you shouldn't use this method most of the time, as we want to simulate
	 * encoder drift in sim.
	 */
	@Logged(name = "actualPose(if run in sim)")
	public Pose2d bestPose() {
		if (RobotBase.isSimulation()) {
			return mapleSim.getSimulatedDriveTrainPose();
		} else {
			return poseEstimate();
		}
	}

	public void resetPose(Pose2d pose) {
		if (RobotBase.isSimulation()) mapleSim.setSimulationWorldPose(pose);
		poseEstimator.resetPose(pose);
	}

	@Logged
	public ChassisSpeeds getMeasuredSpeeds() {
		return ChassisSpeeds.fromRobotRelativeSpeeds(
			getMeasuredSpeedsRobotRelative(),
			offsetWithAlliance(poseEstimate().getRotation())
		);
	}

	@Logged
	public ChassisSpeeds getMeasuredSpeedsRobotRelative() {
		return kinematics.toChassisSpeeds(measuredModuleStates);
	}

	/**
	 * Creates a Command that, when scheduled,
	 * drives the robot forever at the requested x, y, and rotation powers.
	 * Does not move the robot at an exact velocity; so best used in teleop
	 * where exact velocities are not important.
	 * @see InputStream
	 */
	public Command driveCmd(
		InputStream getXPower,
		InputStream getYPower,
		InputStream getRotationPower,
		boolean fieldRelative
	) {
		var maxSpeedMps = config.ofHardware.maxVelocity.in(MetersPerSecond);
		return this.run(() -> {
			var desiredStates = toDesiredModuleStates(
				ChassisSpeeds.fromFieldRelativeSpeeds(
					getXPower.get() * maxSpeedMps,
					getYPower.get() * maxSpeedMps,
					getRotationPower.get() * maxSpeedMps,
					fieldRelative ? offsetWithAlliance(poseEstimate().getRotation()) : Rotation2d.kZero
				),
				true
			);
			for (int i = 0; i < 4; i++) {
				swerveModules[i].setDesiredState(desiredStates[i], false, 0.0);
			}
		}).withName("SwerveDriveCmd(Open Loop)");
	}
	
	@Override
	public Command stopCmd() {
		return this.runOnce(() -> {
			for (int i = 0; i < 4; i++) {
				swerveModules[i].setDriveVoltage(0);
				swerveModules[i].setSteerVoltage(0);
			}
		});
	}
	
	/**
	 * Returns a command that pathfinds the drivetrain to the correct pose.
	 * @param targetPose A function that returns the target pose.
	 * @param flipPoseIfRed Whether to flip the pose if the alliance is red.
	 * @param setpointGenerator Can be null. If not, will use the setpoint generator to generate
	 *                          smoother pathfinding setpoints, reducing slipping.
	 * @return a Command
	 */
	public Command pathfindCmd(
		Pose2d targetPose, boolean flipPoseIfRed,
		@Nullable SwerveSetpointGenerator setpointGenerator
	) {
		// you cannot reassign variables in a lambda function, thus, you must use an AtomicReference
		var currentSetpointRef = new AtomicReference<>(NULL_SETPOINT);
		double maxVelocityMps = config.ofHardware.maxVelocity.in(MetersPerSecond);
		
		return this.run(() -> {
			var realPose = flipPoseIfRed ? AllianceFlipper.flip(targetPose): targetPose;
			repulsor.setGoal(realPose);
			var desiredSpeeds = toDesiredSpeeds(
				repulsor.sampleField(poseEstimate().getTranslation(), maxVelocityMps * .8, 1.5)
			);
			SwerveModuleState[] desiredStates;
			if (setpointGenerator != null) {
				currentSetpointRef.set(
					setpointGenerator.generateSetpoint(currentSetpointRef.get(), desiredSpeeds, 0.02)
				);
				desiredStates = currentSetpointRef.get().moduleStates();
			} else {
				desiredStates = toDesiredModuleStates(desiredSpeeds, true);
			}
			log("setpointGen/speeds", currentSetpointRef.get().robotRelativeSpeeds());
			log("setpointGen/states", currentSetpointRef.get().moduleStates());
			for (int i = 0; i < 4; i++) {
				swerveModules[i].setDesiredState(desiredStates[i], true, 0);
			}
		})
	       .until(() -> repulsor.atGoal(0.1))
	       .withName("PathfindCmd");
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
		return () -> log(
			"headingLockOutput",
			rotationController.calculate(
				poseEstimate().getRotation().getRadians(),
				targetHeading.in(Radians)
			)
		);
	}
	
	public void updateOdometry() {
		Rotation2d latestHeading = getAngleFn.get();
		for (int i = 0; i < 4; i++) {
			measuredModuleStates[i] = swerveModules[i].currentState();
			measuredModulePositions[i] = swerveModules[i].currentPosition();
			if (DriverStation.isDisabled()) {
				swerveModules[i].setDriveVoltage(0.0);
				swerveModules[i].setSteerVoltage(0.0);
			}
		}
		poseEstimator.update(latestHeading, measuredModulePositions);
		singleTagPoseEstimator.update(latestHeading, measuredModulePositions);
		// If robot is rotating too fast, ignore vision observations.
		acceptVisionObservations = latestHeading.minus(prevHeadingCache).getDegrees() / 0.02 < 720.0;
		prevHeadingCache = latestHeading;
	}
	
	public void addVisionData(GlobalPoseEstimate estimate) {
		if (!acceptVisionObservations) return;
		poseEstimator.addVisionMeasurement(estimate.pose(), estimate.timestampSecs(), estimate.standardDeviations());
	}
	
	public void addVisionData(SingleTagPoseEstimate singleTagEstimate) {
		singleTagPoseEstimator.addVisionMeasurement(singleTagEstimate, poseEstimator.getEstimatedPosition());
	}
	
	public void setPathfindingObstacles(List<Pose2d> obstacles) {
		// TODO
	}

	@Override
	public void periodic() {
		if (config.ofControls.poseEstimationMode == PoseEstimationMode.AUTOMATIC) updateOdometry();
	}
	
	@Override
	public void close() {
		for (var mod: swerveModules) { mod.close(); }
	}
}
