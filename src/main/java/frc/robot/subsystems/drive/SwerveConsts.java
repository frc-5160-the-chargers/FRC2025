package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import frc.chargers.misc.TunableValues.TunableNum;
import frc.robot.constants.ChoreoVars;
import frc.robot.constants.TunerConstants;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import static edu.wpi.first.units.Units.*;
import static org.ironmaple.simulation.drivesims.COTS.WHEELS.DEFAULT_NEOPRENE_TREAD;

/**
 * Contains swerve drive constants that aren't already found in the TunerConstants file.
 */
public class SwerveConsts {
    public static final double ODO_FREQUENCY_HZ = 250.0;
    public static final DCMotor DRIVE_MOTOR_TYPE = DCMotor.getKrakenX60(1);
    public static final DCMotor STEER_MOTOR_TYPE = DCMotor.getKrakenX44(1);
    static final Distance BUMPER_WIDTH = Inches.of(3.5);
    static final double KT_AMPS_PER_NM = 1 / (
        DRIVE_MOTOR_TYPE.KtNMPerAmp / TunerConstants.FrontLeft.DriveMotorGearRatio
    );
    // test

    static final TunableNum
        DEMO_POSE_X = new TunableNum("Swerve/demoPose/x", 0),
        DEMO_POSE_Y = new TunableNum("Swerve/demoPose/y", 0),
        DEMO_POSE_HEADING_DEG = new TunableNum("Swerve/demoPose/headingDeg", 0),
        DEMO_DRIVE_VOLTS = new TunableNum("Swerve/demoDriveVolts", 3),
        DEMO_STEER_VOLTS = new TunableNum("Swerve/demoSteerVolts", 3);

    static final TunableNum
        AUTO_KP = new TunableNum("PathFollowing/AutonomousKP", 9),
        REPULSOR_KP = new TunableNum("PathFollowing/TeleopKP", 25),
        ROTATION_KP = new TunableNum("PathFollowing/RotationKP", 8),
        ROTATION_KD = new TunableNum("PathFollowing/RotationKD", 0.02);

    static final SwerveSetpoint NULL_SETPOINT = new SwerveSetpoint(
        new ChassisSpeeds(),
        new SwerveModuleState[]{
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
        },
        DriveFeedforwards.zeros(4)
    );
    static final Translation2d[] MODULE_TRANSLATIONS = {
        new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
        new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
        new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
        new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
    static final Distance DRIVE_BASE_RADIUS = Meters.of(MODULE_TRANSLATIONS[0].getNorm());
    static final DriveTrainSimulationConfig MAPLESIM_CONFIG = DriveTrainSimulationConfig.Default()
        .withRobotMass(ChoreoVars.botMass)
        .withCustomModuleTranslations(MODULE_TRANSLATIONS)
        .withGyro(COTS.ofPigeon2())
        .withBumperSize(
            Meters.of(2 * (TunerConstants.FrontLeft.LocationX + BUMPER_WIDTH.in(Meters))),
            Meters.of(2 * (TunerConstants.FrontLeft.LocationY + BUMPER_WIDTH.in(Meters)))
        )
        .withSwerveModule(new SwerveModuleSimulationConfig(
            DRIVE_MOTOR_TYPE,
            STEER_MOTOR_TYPE,
            TunerConstants.FrontLeft.DriveMotorGearRatio,
            TunerConstants.FrontLeft.SteerMotorGearRatio,
            Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
            Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
            Meters.of(TunerConstants.FrontLeft.WheelRadius),
            KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
            ChoreoVars.cof
        ));
    static final RobotConfig PATH_PLANNER_CONFIG = new RobotConfig(
        ChoreoVars.botMass.in(Kilograms),
        ChoreoVars.botMOI.in(KilogramSquareMeters),
        new ModuleConfig(
            TunerConstants.FrontLeft.WheelRadius,
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
            DEFAULT_NEOPRENE_TREAD.cof,
            DRIVE_MOTOR_TYPE.withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
            TunerConstants.FrontLeft.SlipCurrent,
            1
        ),
        MODULE_TRANSLATIONS
    );

    public static Pose2d getDemoPose() {
        return new Pose2d(
            DEMO_POSE_X.get(), DEMO_POSE_Y.get(),
            Rotation2d.fromDegrees(DEMO_POSE_HEADING_DEG.get())
        );
    }
}
