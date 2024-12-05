package frc.chargers.utils.swerve;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.epilogue.Logged;
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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.chargers.hardware.encoders.EncoderIO;
import frc.chargers.hardware.motorcontrol.MotorIO;
import frc.chargers.utils.Logger;
import frc.chargers.utils.ChassisPowers;
import frc.chargers.utils.commands.SimpleFeedforwardCharacterization;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import java.util.Arrays;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;

import static edu.wpi.first.epilogue.Logged.Strategy.OPT_IN;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj.DriverStation.Alliance;
import static edu.wpi.first.wpilibj.DriverStation.getAlliance;
import static frc.chargers.utils.UtilMethods.pidControllerFrom;

/**
 * A base class for swerve drivetrains.
 * Extend this class for basic swerve functionality.
 */
@SuppressWarnings("unused")
@Logged(strategy = OPT_IN)
public abstract class SwerveDriveBase extends SubsystemBase {
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
		double coefficientOfFriction
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
	
	@Getter @Logged private final SwerveModuleState[] measuredModuleStates = new SwerveModuleState[4];
	@Getter @Logged private SwerveModuleState[] desiredModuleStates = new SwerveModuleState[4];
	@Getter @Logged private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
	private final SwerveModulePosition[] measuredModulePositions = new SwerveModulePosition[4];
	private final Supplier<Rotation2d> getAngleFn;
	
	/**
	 * The logName property should be the same name as the instance variable
	 * which holds the instance of this swerve drive.
	 */
	public SwerveDriveBase(String logName, SwerveDriveConfig config) {
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
			int index = 0;
			for (var sim: mapleSim.getModules()) {
				
				index++;
			}
			this.getAngleFn = mapleSim.getGyroSimulation()::getGyroReading;
			SimulatedArena.getInstance().addDriveTrainSimulation(this.mapleSim);
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
	}
	
	private boolean isRedAlliance() {
		return getAlliance().orElse(Alliance.Blue) == Alliance.Red;
	}
	
	private void logTrajectory(Trajectory<SwerveSample> traj, boolean isStart) {
		if (isRedAlliance()) traj = traj.flipped();
		if (isStart) {
			Logger.log("Choreo/currentTrajectory", traj.getPoses());
			Logger.log("Choreo/currentTrajectoryFinalPose", traj.getFinalPose(false));
		}
	}
	
	public AutoFactory generateAutoFactory() {
		return generateAutoFactory(new AutoFactory.AutoBindings());
	}
	
	public AutoFactory generateAutoFactory(AutoFactory.AutoBindings autoBindings) {
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
	
	@Logged public Pose2d getPose() {
		if (RobotBase.isSimulation()) {
			return this.mapleSim.getSimulatedDriveTrainPose();
		} else {
			return poseEstimator.getEstimatedPosition();
		}
	}
	
	public void resetPose(Pose2d pose) {
		if (RobotBase.isSimulation()) {
			this.mapleSim.setSimulationWorldPose(pose);
		} else {
			poseEstimator.resetPose(pose);
		}
	}
	
	@Logged public ChassisSpeeds getMeasuredSpeeds() {
		return kinematics.toChassisSpeeds(measuredModuleStates);
	}
	
	@Logged public ChassisSpeeds getMeasuredSpeedsRobotRelative() {
		var speeds = getMeasuredSpeeds();
		// TODO change this once ChassisSpeeds becomes immutable
		speeds.toRobotRelativeSpeeds(getAngleFn.get());
		return speeds;
	}
	
	private ChassisSpeeds limitMaxSpeed(ChassisSpeeds speeds) {
		var modStates = kinematics.toSwerveModuleStates(speeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(modStates, config.ofHardware.maxSpeed);
		return kinematics.toChassisSpeeds(modStates);
	}
	
	private void driveCallback(ChassisSpeeds speeds, boolean closedLoop, boolean fieldRelative) {
		// TODO change this when ChassisSpeeds becomes immutable
		if (fieldRelative) {
			speeds.toRobotRelativeSpeeds(getAngleFn.get());
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
	 * Drives the robot forever at the requested chassis powers.
	 * Does not move the robot at an exact velocity; so best used in teleop
	 * where exact velocities are not important.
	 */
	public Command teleopDriveCmd(Supplier<ChassisPowers> getChassisPowers, boolean fieldRelative) {
		var maxSpeedMps = config.ofHardware.maxSpeed.in(MetersPerSecond);
		return this.run(() -> {
			var power = getChassisPowers.get();
			driveCallback(
				new ChassisSpeeds(
					power.xPower() * maxSpeedMps,
					power.yPower() * maxSpeedMps,
					power.rotationPower() * maxSpeedMps
				),
				false, fieldRelative
			);
		});
	}
	
	/** Drives the robot forever at the exact chassis speeds requested. */
	public Command velocityDriveCmd(Supplier<ChassisSpeeds> getChassisSpeeds, boolean fieldRelative) {
		return this.run(() -> driveCallback(getChassisSpeeds.get(), true, fieldRelative));
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
		);
	}
	
	@Override
	public void periodic() {
		for (int i = 0; i < 4; i++) {
			measuredModuleStates[i] = swerveModules[i].currentState();
			measuredModulePositions[i] = swerveModules[i].currentPosition();
			swerveModules[i].periodic();
		}
		poseEstimator.update(getAngleFn.get(), measuredModulePositions);
		simpleFieldViz.setRobotPose(getPose());
	}
}
