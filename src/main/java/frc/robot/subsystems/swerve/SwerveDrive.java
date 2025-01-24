package frc.robot.subsystems.swerve;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.util.ChoreoAllianceFlipUtil;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
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
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.chargers.hardware.encoders.Encoder;
import frc.chargers.hardware.encoders.VoidEncoder;
import frc.chargers.hardware.motorcontrol.Motor;
import frc.chargers.utils.InputStream;
import frc.chargers.utils.PIDConstants;
import frc.chargers.utils.RepulsorFieldPlanner;
import frc.chargers.utils.UtilMethods;
import frc.robot.subsystems.StandardSubsystem;
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
import static org.photonvision.PhotonUtils.getYawToPose;

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
		double forceKT
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

	@Getter private final SwerveDriveConfig config;
	@Getter private final SwerveDriveKinematics kinematics;
	private final SwerveDrivePoseEstimator poseEstimator;
	private final SwerveDriveSimulation mapleSim;
	@Getter private final SwerveModule[] swerveModules = new SwerveModule[4];
	private final RepulsorFieldPlanner repulsor = new RepulsorFieldPlanner();
	
	@Logged private boolean acceptVisionObservations = true;
	@Logged private final SwerveModuleState[] measuredModuleStates = new SwerveModuleState[4];
	private final SwerveModulePosition[] measuredModulePositions = new SwerveModulePosition[4];

	private final PIDController xPoseController;
	private final PIDController yPoseController;
	private final PIDController rotationController;

	private final Supplier<Rotation2d> getAngleFn;
	private Rotation2d prevHeadingCache = Rotation2d.kZero;
	private Pose2d estimatedPose = Pose2d.kZero;
	private final double[] stdDevStore = new double[3];

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

		var driveSimConfig =
			DriveTrainSimulationConfig.Default()
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
				steerMotor = moduleSim.useSteerMotorController(new SwerveModule.DummyMotor(null));
				driveMotor = moduleSim.useDriveMotorController(new SwerveModule.DummyMotor(null));
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
			this.getAngleFn = mapleSim.getGyroSimulation()::getGyroReading;
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

	private void choreoCallback(SwerveSample trajSample) {
		var vx = trajSample.vx + xPoseController.calculate(getPose().getX(), trajSample.x);
		var vy = trajSample.vy + yPoseController.calculate(getPose().getY(), trajSample.y);
		var rotationV = trajSample.omega + rotationController.calculate(
			angleModulus(getPose().getRotation().getRadians()),
			angleModulus(trajSample.heading)
		);
		var desiredModuleStates = getDesiredStates(
			ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rotationV, getPose().getRotation())
		);
		for (int i = 0; i < 4; i++) {
			var desiredState = desiredModuleStates[i];
			double torqueFF = 0.0;
			if (config.ofControls.forceKT != 0.0) {
				double wheelTorqueNm = config.ofModules.wheelRadius.in(Meters) * (
					desiredState.angle.getCos() * trajSample.moduleForcesX()[i] +
					desiredState.angle.getSin() * trajSample.moduleForcesY()[i]
				);
				torqueFF = wheelTorqueNm / config.ofModules.driveGearRatio * config.ofControls.forceKT;
			}
			swerveModules[i].setDesiredState(desiredState, true, torqueFF);
		}
	}

	@Logged private List<SwerveSample> trajSamples = List.of();

	public AutoFactory createAutoFactory() {
		return new AutoFactory(
			this::getPose,
			this::resetPose,
			this::choreoCallback,
			true,
			this,
			(trajectory, isStart) -> {
				trajSamples = trajectory.samples();
				log("currentTrajectory/name", trajectory.name());
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

	public void resetPose(Pose2d pose) {
		if (RobotBase.isSimulation()) mapleSim.setSimulationWorldPose(pose);
		poseEstimator.resetPose(pose);
	}

	@Logged
	public ChassisSpeeds getMeasuredSpeeds() {
		return ChassisSpeeds.fromRobotRelativeSpeeds(
			getMeasuredSpeedsRobotRelative(),
			offsetWithAlliance(getPose().getRotation())
		);
	}

	@Logged
	public ChassisSpeeds getMeasuredSpeedsRobotRelative() {
		return kinematics.toChassisSpeeds(measuredModuleStates);
	}
	
	private SwerveModuleState[] getDesiredStates(ChassisSpeeds speeds) {
		speeds = ChassisSpeeds.discretize(speeds, 0.02);
		log("desiredSpeeds", speeds);
		
		var desiredStates = kinematics.toSwerveModuleStates(speeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, config.ofHardware.maxVelocity);
		for (int i = 0; i < 4; i++) {
			var currentAngle = swerveModules[i].getSteerAngle();
			desiredStates[i].optimize(currentAngle);
			desiredStates[i].cosineScale(currentAngle);
		}
		log("desiredStates", desiredStates);
		return desiredStates;
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
		return this.run(() -> {
			var desiredStates = getDesiredStates(
				ChassisSpeeds.fromFieldRelativeSpeeds(
					getXPower.get() * maxSpeedMps,
					getYPower.get() * maxSpeedMps,
					getRotationPower.get() * maxSpeedMps,
					fieldRelative ? offsetWithAlliance(getPose().getRotation()) : Rotation2d.kZero
				)
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
	
	public Command pathfindCmd(Supplier<Pose2d> targetPose) {
		return this.run(() -> {
			var targetPoseVal = targetPose.get();
			repulsor.setGoal(targetPoseVal.getTranslation());
			var goal = repulsor.getRequest(
				getPose(), config.ofHardware.maxVelocity.in(MetersPerSecond),
				true, targetPoseVal.getRotation()
			);
			choreoCallback(goal);
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
				getPose().getRotation().getRadians(),
				targetHeading.in(Radians)
			)
		);
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
				actualPose.set(shouldFlip ? ChoreoAllianceFlipUtil.flip(pose) : pose);
			}
			double output = rotationController.calculate(
				getPose().getRotation().getRadians(),
				getYawToPose(getPose(), actualPose.get()).getRadians()
			);
			log("headingLockOutput", output);
			return output;
		};
	}
	
	/**
	 * Enables high-frequency odometry on this drivetrain.
	 * <h4>You must enable high-frequency updating on your drive and steer motors yourself!</h4>
	 */
	public void enableHighFrequencyOdometry(TimedRobot robot, Time period) {
		useHighFrequencyOdometry = true;
		robot.addPeriodic(this::updateOdometry, period);
	}
	
	private boolean useHighFrequencyOdometry = false;
	private void updateOdometry() {
		Rotation2d latestHeading = getAngleFn.get();
		for (int i = 0; i < 4; i++) {
			measuredModuleStates[i] = swerveModules[i].currentState();
			measuredModulePositions[i] = swerveModules[i].currentPosition();
			if (DriverStation.isDisabled()) {
				swerveModules[i].setDriveVoltage(0.0);
				swerveModules[i].setSteerVoltage(0.0);
			}
		}
		estimatedPose = poseEstimator.update(latestHeading, measuredModulePositions);
		// If robot is rotating too fast, ignore vision observations.
		acceptVisionObservations = latestHeading.minus(prevHeadingCache).getDegrees() / 0.02 < 720.0;
		prevHeadingCache = latestHeading;
	}
	
	public void addVisionPoseEstimate(Pose2d visionPose, double timestampSecs, Matrix<N3, N1> standardDeviations) {
		if (!acceptVisionObservations) return;
		poseEstimator.addVisionMeasurement(visionPose, timestampSecs, standardDeviations);
	}
	
	public void setPathfindingObstacles(List<Pose2d> obstacles) {
		// TODO
	}

	@Override
	public void periodic() { if (!useHighFrequencyOdometry) updateOdometry(); }
	
	@Override
	public void close() {
		for (var mod: swerveModules) { mod.close(); }
	}
}
