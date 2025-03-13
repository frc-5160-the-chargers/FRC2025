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
import edu.wpi.first.math.filter.SlewRateLimiter;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.chargers.hardware.encoders.Encoder;
import frc.chargers.hardware.encoders.VoidEncoder;
import frc.chargers.hardware.motorcontrol.Motor;
import frc.chargers.utils.AllianceUtil;
import frc.chargers.utils.Tracer;
import frc.chargers.utils.data.InputStream;
import frc.chargers.utils.data.PIDConstants;
import frc.chargers.utils.RepulsorFieldPlanner;
import frc.chargers.utils.data.TunableValues.TunableNum;
import frc.robot.components.vision.GlobalPoseEstimate;
import frc.robot.components.vision.SingleTagPoseEstimate;
import frc.robot.components.vision.SingleTagPoseEstimator;
import frc.robot.subsystems.StandardSubsystem;
import lombok.RequiredArgsConstructor;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.jetbrains.annotations.Nullable;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;
import java.util.function.Supplier;

import static edu.wpi.first.math.MathUtil.angleModulus;
import static edu.wpi.first.math.util.Units.metersToInches;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * A drivetrain with 4 drive motors and 4 steer motors.
 * Each steer motor can control the exact position of each drive motor,
 * allowing for omnidirectional movement and driving while turning. <br/>
 *
 * Note: for pose estimation to work, you must call drivetrain.updateOdometry()
 * in robotPeriodic or through an addPeriodic() call for higher frequency.
 */
public class SwerveDrive extends StandardSubsystem {
	public record SwerveMotorConfig(
		Function<SwerveCorner, Motor> realDriveMotorCreator,
		Function<SwerveCorner, Motor> realSteerMotorCreator,
		Function<SwerveCorner, Encoder> realAbsEncoderCreator,
		@Nullable TalonFXConfiguration simSteerConfig,
		@Nullable TalonFXConfiguration simDriveConfig
	) {}

	/**
	 * Use DEFAULT_NEOPRENE_TREAD.cof or COLSONS.cof for coefficientOfFriction,
	 * depending on wheel type.
	 */
	public record SwerveHardwareSpecs(
		Distance trackWidth,
		Distance wheelBase,
		DCMotor driveMotorType,
		DCMotor steerMotorType,
		LinearVelocity maxVelocity,
		double coefficientOfFriction,
		Mass robotMass,
		Distance widthOfBumpers
	){
		public Distance drivebaseRadius() {
			return Meters.of(Math.hypot(trackWidth.in(Meters) / 2, wheelBase.in(Meters) / 2));
		}
	}
	
	public record SwerveControlsConfig(
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
		SwerveX2L2P11(6.20, 12.1 * 0.775, Inches.of(2)); // Docs say its 12.1 - i think the hardware team messed up
		
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
	
	// static members
	private static final SwerveSetpoint NULL_SETPOINT = new SwerveSetpoint(
		new ChassisSpeeds(),
		new SwerveModuleState[]{ new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState() },
		DriveFeedforwards.zeros(4)
	);
	private static final TunableNum DEMO_POSE_X = new TunableNum("swerveDrive/demoPose/x", 0);
	private static final TunableNum DEMO_POSE_Y = new TunableNum("swerveDrive/demoPose/y", 0);
	private static final TunableNum DEMO_POSE_HEADING_DEG = new TunableNum("swerveDrive/demoPose/headingDeg", 0);
	private static final TunableNum DEMO_DRIVE_VOLTS = new TunableNum("swerveDrive/demoDriveVolts", 3);
	private static final TunableNum DEMO_STEER_VOLTS = new TunableNum("swerveDrive/demoSteerVolts", 3);

	// instance members
	public final SwerveHardwareSpecs hardwareSpecs;
	public final SwerveDriveKinematics kinematics;
	public final SwerveModule[] swerveModules = new SwerveModule[4];
	private final SwerveControlsConfig controlsConfig;
	private final ModuleType moduleType;
	private final SwerveDrivePoseEstimator poseEstimator;
	private final SingleTagPoseEstimator singleTagPoseEstimator; // isn't working yet -- don't use
	private final SwerveDriveSimulation mapleSim;
	private final RepulsorFieldPlanner repulsor = new RepulsorFieldPlanner();
	
	@Logged private boolean acceptVisionObservations = true;
	@Logged private final SwerveModuleState[] measuredModuleStates = new SwerveModuleState[4];
	private final SwerveModulePosition[] measuredModulePositions = new SwerveModulePosition[4];

	@Logged private final PIDController xPoseController;
	@Logged private final PIDController yPoseController;
	@Logged private final PIDController rotationController;

	private final Supplier<Rotation2d> gyroYawSupplier;
	private Rotation2d prevHeadingCache = Rotation2d.kZero;
	@Logged(name = "singleTagEstimation/id")
	private int targetTagId = -1;
	private boolean useSingleTagEstimation = false;
	@Logged private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();

	// workaround before array logging comes around
	@Logged private final SwerveModule topLeftModule;
	@Logged private final SwerveModule topRightModule;
	@Logged private final SwerveModule bottomLeftModule;
	@Logged private final SwerveModule bottomRightModule;
	
	/** A SysId routine for characterization translational velocity/accel. */
	public final SysIdRoutine translationSysIdRoutine;
	/** A SysId routine for characterization rotational velocity/accel. */
	public final SysIdRoutine rotationSysIdRoutine;
	
	public SwerveDrive(
		SwerveHardwareSpecs hardwareSpecs, 
		SwerveControlsConfig controlsConfig,
		ModuleType moduleType,
		SwerveMotorConfig motorConfig,
		Supplier<Rotation2d> gyroYawSupplier
	) {
		Arrays.fill(measuredModuleStates, new SwerveModuleState());
		Arrays.fill(measuredModulePositions, new SwerveModulePosition());
		
		this.hardwareSpecs = hardwareSpecs;
		this.controlsConfig = controlsConfig;
		this.moduleType = moduleType;
		this.kinematics = new SwerveDriveKinematics(
			new Translation2d(
				hardwareSpecs.trackWidth.div(2),
				hardwareSpecs.wheelBase.div(2)
			),
			new Translation2d(
				hardwareSpecs.trackWidth.div(2),
				hardwareSpecs.wheelBase.div(-2)
			),
			new Translation2d(
				hardwareSpecs.trackWidth.div(-2),
				hardwareSpecs.wheelBase.div(2)
			),
			new Translation2d(
				hardwareSpecs.trackWidth.div(-2),
				hardwareSpecs.wheelBase.div(-2)
			)
		);
		this.poseEstimator = new SwerveDrivePoseEstimator(
			kinematics, Rotation2d.kZero, measuredModulePositions, Pose2d.kZero
		);
		this.singleTagPoseEstimator = new SingleTagPoseEstimator(kinematics, VecBuilder.fill(0.003, 0.003, 0.02));

		var driveSimConfig =
			DriveTrainSimulationConfig.Default()
				.withTrackLengthTrackWidth(hardwareSpecs.wheelBase, hardwareSpecs.trackWidth)
				.withRobotMass(hardwareSpecs.robotMass)
				.withGyro(COTS.ofPigeon2())
				.withBumperSize(
					hardwareSpecs.wheelBase.plus(hardwareSpecs.widthOfBumpers.times(2)),
					hardwareSpecs.trackWidth.plus(hardwareSpecs.widthOfBumpers.times(2))
				)
				.withSwerveModule(
					new SwerveModuleSimulationConfig(
						hardwareSpecs.driveMotorType,
						hardwareSpecs.steerMotorType,
						moduleType.driveGearRatio,
						moduleType.steerGearRatio,
						Volts.of(0.1),
						Volts.of(0.2),
						moduleType.wheelRadius,
						KilogramSquareMeters.of(0.004),
						hardwareSpecs.coefficientOfFriction
					)
				);

		this.mapleSim = new SwerveDriveSimulation(driveSimConfig, Pose2d.kZero);
		this.translationSysIdRoutine = new SysIdRoutine(
			new SysIdRoutine.Config(
				Volts.per(Second).of(0.5),
				Volts.of(3),
				Seconds.of(10),
				state -> log("sysIdState", state.toString())
			),
			new SysIdRoutine.Mechanism(
				voltage -> {
					for (var module: swerveModules) {
						module.setDriveVoltage(voltage.in(Volts));
						module.setSteerAngle(Rotation2d.kZero);
					}
				},
				log -> {},
				this
			)
		);
		this.rotationSysIdRoutine = new SysIdRoutine(
			new SysIdRoutine.Config(
				Volts.per(Second).of(0.5),
				Volts.of(3),
				Seconds.of(10),
				state -> log("sysIdState", state.toString())
			),
			new SysIdRoutine.Mechanism(
				voltage -> {
					for (var module: swerveModules) {
						module.setDriveVoltage(voltage.in(Volts));
						module.setSteerAngle(Rotation2d.kZero);
					}
				},
				log -> {},
				this
			)
		);
		this.xPoseController = controlsConfig.pathTranslationPID.asController();
		this.yPoseController = controlsConfig.pathTranslationPID.asController();
		this.rotationController = controlsConfig.pathRotationPID.asController();
		this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
		
		this.xPoseController.setTolerance(0.01);
		this.yPoseController.setTolerance(0.01);
		this.rotationController.setTolerance(0.05);
		
		for (int i = 0; i < 4; i++) {
			Motor steerMotor;
			Motor driveMotor;
			Encoder absoluteEncoder;
			if (RobotBase.isSimulation()) {
				var moduleSim = mapleSim.getModules()[i];
				// useSteerMotorController and useDriveMotorController return the passed in DummyMotor.
				steerMotor = moduleSim.useSteerMotorController(new SwerveModule.DummyMotor(motorConfig.simSteerConfig));
				driveMotor = moduleSim.useDriveMotorController(new SwerveModule.DummyMotor(motorConfig.simDriveConfig));
				absoluteEncoder = new VoidEncoder();
			} else {
				// SwerveCorner.values() is TL, TR, BL, and BR(enums are defined in this order)
				var corner = SwerveCorner.values()[i];
				steerMotor = motorConfig.realSteerMotorCreator.apply(corner);
				driveMotor = motorConfig.realDriveMotorCreator.apply(corner);
				absoluteEncoder = motorConfig.realAbsEncoderCreator.apply(corner);
			}
			
			steerMotor.setControlsConfig(
				Motor.ControlsConfig.EMPTY
					.withGearRatio(moduleType.steerGearRatio)
					.withPositionPID(controlsConfig.steerPID)
					.withContinuousInput(true)
			);
			driveMotor.setControlsConfig(
				Motor.ControlsConfig.EMPTY
					.withGearRatio(moduleType.driveGearRatio)
					.withVelocityPID(controlsConfig.velocityPID)
			);
			
			this.swerveModules[i] = new SwerveModule(
				driveMotor, steerMotor, absoluteEncoder,
				moduleType.wheelRadius,
				hardwareSpecs.maxVelocity,
				controlsConfig.velocityFeedforward
			);
		}
		if (RobotBase.isSimulation()) {
			// assuming perfect gyro
			this.gyroYawSupplier = () -> mapleSim.getSimulatedDriveTrainPose().getRotation();
			SimulatedArena.getInstance().addDriveTrainSimulation(mapleSim);
		} else {
			this.gyroYawSupplier = gyroYawSupplier;
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
		if (limitVelocity) SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, hardwareSpecs.maxVelocity);
		for (int i = 0; i < 4; i++) {
			var currentAngle = swerveModules[i].getSteerAngle();
			desiredStates[i].optimize(currentAngle);
			desiredStates[i].cosineScale(currentAngle);
		}
		log("desiredStates", desiredStates);
		return desiredStates;
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
			(SwerveSample trajSample) -> {
				var desiredModuleStates = toDesiredModuleStates(toDesiredSpeeds(trajSample, 1), false);
				for (int i = 0; i < 4; i++) {
					var desiredState = desiredModuleStates[i];
					double wheelTorqueNm = moduleType.wheelRadius.in(Meters) * (
						desiredState.angle.getCos() * trajSample.moduleForcesX()[i] +
							desiredState.angle.getSin() * trajSample.moduleForcesY()[i]
					);
					double torqueFF = wheelTorqueNm / moduleType.driveGearRatio * controlsConfig.forceKT;
					swerveModules[i].setDesiredState(desiredState, true, torqueFF);
				}
			},
			true,
			this,
			(trajectory, isStart) -> {
				log("currentTrajectory/samples", trajectory.samples(), SwerveSample.struct);
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
				hardwareSpecs.robotMass,
				bodyMoi,
				new ModuleConfig(
					moduleType.wheelRadius,
					hardwareSpecs.maxVelocity,
					hardwareSpecs.coefficientOfFriction,
					hardwareSpecs.driveMotorType(),
					moduleType.driveGearRatio,
					driveCurrentLimit,
					4
				),
				kinematics.getModules()
			),
			hardwareSpecs.steerMotorType.freeSpeedRadPerSec
		);
	}
	
	public Command runDriveMotors() {
		return this.run(() -> {
			double driveVolts = DEMO_DRIVE_VOLTS.get();
			for (var mod: swerveModules) {
				mod.setDriveVoltage(driveVolts);
				mod.setSteerAngle(Rotation2d.kZero);
			}
		}).withName("RunDriveMotors");
	}
	
	public Command runTurnMotors() {
		return this.run(() -> {
			double steerVolts = DEMO_STEER_VOLTS.get();
			for (var mod: swerveModules) {
				mod.setSteerVoltage(steerVolts);
			}
		}).withName("RunTurnMotors");
	}
	
	public Command setSteerAngles(Rotation2d... angles) {
		if (angles.length == 1) {
			return this.run(() -> {
				for (var mod: swerveModules) {
					mod.setSteerAngle(angles[0]);
				}
			}).withName("SetSteerAngles");
		} else if (angles.length == 4) {
			return this.run(() -> {
				for (int i = 0; i < 4; i++) {
					swerveModules[i].setSteerAngle(angles[i]);
				}
			});
		} else {
			return Commands.print("Invalid amount of steer angles - command ignored.");
		}
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
		if (RobotBase.isSimulation()) {
			// don't reset rotation in sim, as maple sim's heading has already changed
			poseEstimator.resetTranslation(pose.getTranslation());
			mapleSim.setSimulationWorldPose(pose);
		} else {
			poseEstimator.resetPose(pose);
		}
	}
	
	public void resetToDemoPose() {
		resetPose(
			new Pose2d(
				DEMO_POSE_X.get(), DEMO_POSE_Y.get(),
				Rotation2d.fromDegrees(DEMO_POSE_HEADING_DEG.get())
			)
		);
	}
	
	public double getOverallSpeedMPS() {
		return Math.hypot(
			robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond
		);
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
		var maxSpeedMps = hardwareSpecs.maxVelocity.in(MetersPerSecond);
		return this.run(() -> {
			var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
				forwardOutput.get() * maxSpeedMps,
				strafeOutput.get() * maxSpeedMps,
				rotationOutput.get() * maxSpeedMps,
				fieldRelative ? offsetWithAlliance(poseEstimate().getRotation()) : Rotation2d.kZero
			);
			if (Math.abs(speeds.vxMetersPerSecond) < 0.05 && Math.abs(speeds.vyMetersPerSecond) < 0.05 && Math.abs(speeds.omegaRadiansPerSecond) < 0.05) {
				requestStop();
				return;
			}
			var desiredStates = toDesiredModuleStates(speeds, true);
			for (int i = 0; i < 4; i++) {
				swerveModules[i].setDesiredState(desiredStates[i], false, 0.0);
			}
		}).withName("SwerveDriveCmd(Open Loop)");
	}
	
	@Override
	protected void requestStop() {
		for (int i = 0; i < 4; i++) {
			swerveModules[i].setDriveVoltage(0);
			swerveModules[i].setSteerVoltage(0);
		}
	}
	
	/**
	 * Returns a Command that simply aligns to a pose, with no obstacle avoidance.
	 * @param blueTargetPose The target pose on the blue side of the field
	 * @param flipPoseIfRed Whether to flip the pose if the alliance is red.
	 * @return A command
	 */
	public Command alignCmd(Pose2d blueTargetPose, boolean flipPoseIfRed) {
		return this.run(() -> {
			var target = flipPoseIfRed ? AllianceUtil.flipIfRed(blueTargetPose): blueTargetPose;
			log("align/goal", target);
			var vx = xPoseController.calculate(poseEstimate().getX(), target.getX()) / 2;
			var vy = yPoseController.calculate(poseEstimate().getY(), target.getY()) / 2;
			var rotationV = rotationController.calculate(
				angleModulus(bestPose().getRotation().getRadians()),
				angleModulus(target.getRotation().getRadians())
			);
			var desiredStates = toDesiredModuleStates(
				ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rotationV, bestPose().getRotation()),
				true
			);
			for (int i = 0; i < 4; i++) {
				swerveModules[i].setDesiredState(desiredStates[i], true, 0);
			}
		})
	       .until(
			   () -> log("atSetpoints", xPoseController.atSetpoint() &&
			         yPoseController.atSetpoint() &&
			         rotationController.atSetpoint())
	       )
	       .andThen(stopCmd())
	       .withName("align(drivetrain)");
	}
	
	/**
	 * Returns a command that pathfinds the drivetrain to the correct pose.
	 * @param blueTargetPose A function that returns the target pose.
	 * @param flipPoseIfRed Whether to flip the pose if the alliance is red.
	 * @param setpointGenerator Can be null. If not, will use the setpoint generator to generate
	 *                          smoother pathfinding setpoints, reducing inaccuracy in pose estimation.
	 * @return a Command
	 */
	public Command pathfindCmd(
		Pose2d blueTargetPose, boolean flipPoseIfRed,
		@Nullable SwerveSetpointGenerator setpointGenerator
	) {
		// you cannot reassign variables in a lambda function, thus, you must use an AtomicReference
		var currentSetpointRef = new AtomicReference<>(NULL_SETPOINT);
		double maxVelocityMps = hardwareSpecs.maxVelocity.in(MetersPerSecond);
		
		return this.run(() -> {
			Tracer.startTrace("repulsor pathfinding");
			var target = flipPoseIfRed ? AllianceUtil.flipIfRed(blueTargetPose): blueTargetPose;
			repulsor.setGoal(target);
			var sample = repulsor.sampleField(poseEstimate().getTranslation(), maxVelocityMps * .8, 1.5);
			var desiredSpeeds = toDesiredSpeeds(sample, 1.8);
			SwerveModuleState[] desiredStates;
			if (setpointGenerator != null) {
				currentSetpointRef.set(
					setpointGenerator.generateSetpoint(currentSetpointRef.get(), desiredSpeeds, 0.02)
				);
				desiredStates = currentSetpointRef.get().moduleStates();
			} else {
				desiredStates = toDesiredModuleStates(desiredSpeeds, true);
			}
			log("pathfinding/goal", target);
			log("setpointGen/speeds", currentSetpointRef.get().robotRelativeSpeeds());
			log("setpointGen/states", currentSetpointRef.get().moduleStates());
			for (int i = 0; i < 4; i++) {
				swerveModules[i].setDesiredState(desiredStates[i], true, 0);
			}
			Tracer.endTrace();
		})
	       .until(() -> repulsor.atGoal(0.02))
	       .andThen(super.stopCmd())
	       .withName("PathfindCmd");
	}
	
	/**
	 * Drives the robot while locking to a specific heading.
	 */
	public Command driveWithAimCmd(
		InputStream forwardOutput,
		InputStream strafeOutput,
		Angle targetHeading,
		double maxRotationSpeed,
		boolean fieldRelative
	) {
		return driveCmd(
			forwardOutput, strafeOutput,
			() -> {
				double currentHeading = bestPose().getRotation().getRadians();
				double output =
					rotationController.calculate(currentHeading, targetHeading.in(Radians))
						/ hardwareSpecs.maxVelocity.in(MetersPerSecond);
				output = Math.max(output, maxRotationSpeed);
				log("targetHeading", targetHeading);
				log("aimingSpeed", output);
				return output;
			},
			fieldRelative
		).withName("drive with aim");
	}
	
	public void refreshData() {
		for (int i = 0; i < 4; i++) {
			measuredModuleStates[i] = swerveModules[i].currentState();
			measuredModulePositions[i] = swerveModules[i].currentPosition();
		}
		robotRelativeSpeeds = kinematics.toChassisSpeeds(measuredModuleStates);
	}
	
	/** Must be called periodically at whatever frequency you want. */
	public void updateOdometry() {
		Rotation2d latestHeading = gyroYawSupplier.get();
		refreshData();
		poseEstimator.update(latestHeading, measuredModulePositions);
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
		if (DriverStation.isDisabled()) requestStop();
	}
	
	@Override
	public void close() {
		for (var mod: swerveModules) { mod.close(); }
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
				this.run(() -> {
					double speed = limiter.calculate(0.25);
					var desiredStates = toDesiredModuleStates(new ChassisSpeeds(0, 0, speed), false);
					for (int i = 0; i < 4; i++) {
						swerveModules[i].setDesiredState(desiredStates[i], true, 0);
					}
				})
			),
			// Measurement sequence
			Commands.sequence(
				// Wait for modules to fully orient before starting measurement
				Commands.waitSeconds(1.0),
				// Record starting measurement
				Commands.runOnce(
					() -> {
						for (int i = 0; i < 4; i++) {
							state.positions[i] = swerveModules[i].getRawDrivePositionRad();
						}
						state.lastAngle = gyroYawSupplier.get();
						state.gyroDelta = 0.0;
					}),
				
				// Update gyro delta
				Commands.run(
						() -> {
							var rotation = gyroYawSupplier.get();
							state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
							state.lastAngle = rotation;
							
							double[] positions = new double[4];
							for (int i = 0; i < 4; i++) {
								positions[i] = swerveModules[i].getRawDrivePositionRad();
							}
							double wheelDelta = 0.0;
							for (int i = 0; i < 4; i++) {
								wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
							}
							double wheelRadius =
								(state.gyroDelta * hardwareSpecs.drivebaseRadius().in(Meters)) / wheelDelta;
							
							log("wheelRadiusCharacterization/delta", wheelDelta);
							log("wheelRadiusCharacterization/found wheel radius(meters)", wheelRadius);
						})
					
					// When cancelled, calculate and print results
					.finallyDo(
						() -> {
							double[] positions = new double[4];
							for (int i = 0; i < 4; i++) {
								positions[i] = swerveModules[i].getRawDrivePositionRad();
							}
							double wheelDelta = 0.0;
							for (int i = 0; i < 4; i++) {
								wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
							}
							double wheelRadius =
								(state.gyroDelta * hardwareSpecs.drivebaseRadius().in(Meters)) / wheelDelta;
							
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
